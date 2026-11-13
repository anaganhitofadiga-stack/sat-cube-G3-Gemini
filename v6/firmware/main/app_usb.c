/*
 * app_usb.c — consola de telecomandos e telemetria CSV sobre o cabo USB.
 */

#include "app_usb.h"
#include "app_controlo.h"
#include "controlo_atitude.h"
#include "controlo_visual.h"
#include "hal_cubo_hw.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/usb_serial_jtag.h"

/* O nome do cabecalho da ligacao ao VFS mudou no ESP-IDF 5.3. */
#if __has_include("driver/usb_serial_jtag_vfs.h")
#include "driver/usb_serial_jtag_vfs.h"
#define USB_LIGAR_VFS() usb_serial_jtag_vfs_use_driver()
#elif __has_include("esp_vfs_usb_serial_jtag.h")
#include "esp_vfs_usb_serial_jtag.h"
#define USB_LIGAR_VFS() esp_vfs_usb_serial_jtag_use_driver()
#else
#define USB_LIGAR_VFS() do { } while (0)
#endif

static const char *TAG = "usb";

#define USB_RX_BUFFER       512
#define USB_TX_BUFFER       2048
#define USB_LINHA_MAX       64
#define RAD_PARA_GRAUS      57.2957795131f
#define RAD_S_PARA_RPM      9.54929658551f

static int telemetria_ligada = 1;
static float ultimo_yaw_graus;

static void imprimir_ajuda(void) {
    printf(
        "\n# ---- Cubo V5, consola de bancada ----\n"
        "#  a          armar\n"
        "#  d          desarmar\n"
        "#  x          PARAGEM DE EMERGENCIA (so sai com R ou reset)\n"
        "#  m <0|1|2>  modo: 0 parado, 1 atitude pela consola, 2 visual\n"
        "#  y <graus>  yaw alvo absoluto (modo 1)\n"
        "#  s <graus>  yaw alvo = yaw atual + graus (degrau relativo)\n"
        "#  z          yaw alvo = yaw atual\n"
        "#  o <graus>  offset do alvo visual (nivel 7)\n"
        "#  t          ligar/desligar a telemetria\n"
        "#  c          cabecalho das colunas\n"
        "#  i          estado do hardware\n"
        "#  R          reiniciar a placa\n"
        "#  ?          esta ajuda\n");
    fflush(stdout);
}

static void imprimir_cabecalho(void) {
    printf("# T,t_ms,modo,armado,vis,falhas,roll_deg,pitch_deg,yaw_deg,"
           "alvo_deg,erro_deg,wx_dps,wy_dps,wz_dps,vel_des_dps,torque_Nm,"
           "roda_rpm,dt_ms,lim,img_ok,img_rej\n");
    fflush(stdout);
}

static void imprimir_estado(void) {
    printf("# motor_ativo=%d erros_i2c=%lu erros_can=%lu falha_moteus=%d\n",
           hal_cubo_motor_ativo(),
           (unsigned long)hal_cubo_erros_i2c(),
           (unsigned long)hal_cubo_erros_can(),
           hal_cubo_falha_moteus());
    fflush(stdout);
}

static int enviar_tc(uint8_t tipo, float valor) {
    TelecomandoControlo tc = {.tipo = tipo, .valor = valor};
    return app_controlo_enviar_telecomando(&tc);
}

static void imprimir_telemetria(const TelemetriaControlo *t) {
    float erro = envolver_angulo(t->yaw_alvo - t->yaw) * RAD_PARA_GRAUS;
    float roda_rpm = isfinite(t->vel_roda) ? t->vel_roda * RAD_S_PARA_RPM
                                           : NAN;
    ultimo_yaw_graus = t->yaw * RAD_PARA_GRAUS;
    printf("T,%lld,%u,%u,%u,%lu,"
           "%.2f,%.2f,%.2f,%.2f,%.2f,"
           "%.2f,%.2f,%.2f,%.2f,%.4f,%.1f,%.2f,%u,%lu,%lu\n",
           (long long)(t->t_us / 1000),
           (unsigned)t->modo, (unsigned)t->armado, (unsigned)t->estado_visual,
           (unsigned long)t->falhas,
           t->roll * RAD_PARA_GRAUS, t->pitch * RAD_PARA_GRAUS,
           t->yaw * RAD_PARA_GRAUS, t->yaw_alvo * RAD_PARA_GRAUS, erro,
           t->wx * RAD_PARA_GRAUS, t->wy * RAD_PARA_GRAUS,
           t->wz * RAD_PARA_GRAUS,
           t->vel_corpo_desejada * RAD_PARA_GRAUS,
           t->torque_roda, roda_rpm, t->dt * 1000.0f,
           (unsigned)t->limitado,
           (unsigned long)t->imagens_aceites,
           (unsigned long)t->imagens_rejeitadas);
}

static void processar_linha(char *linha) {
    while (*linha == ' ' || *linha == '\t') {
        linha++;
    }
    if (*linha == '\0' || *linha == '#') {
        return;
    }
    char comando = *linha;
    const char *argumento = linha + 1;
    float valor = strtof(argumento, NULL);

    switch (comando) {
    case 'a':
        printf("# armar: %s\n", enviar_tc(TC_ARMAR, 0.0f) ? "aceite" : "fila cheia");
        break;
    case 'd':
        enviar_tc(TC_DESARMAR, 0.0f);
        printf("# desarmado\n");
        break;
    case 'x':
        enviar_tc(TC_PARAGEM_EMERGENCIA, 0.0f);
        printf("# PARAGEM DE EMERGENCIA — usar R (ou reset) para voltar\n");
        break;
    case 'm':
        enviar_tc(TC_MODO, valor);
        printf("# modo -> %d\n", (int)valor);
        break;
    case 'y':
        enviar_tc(TC_YAW_ALVO_GRAUS, valor);
        printf("# yaw alvo -> %.2f graus\n", valor);
        break;
    case 's':
        enviar_tc(TC_YAW_ALVO_GRAUS, ultimo_yaw_graus + valor);
        printf("# degrau de %.2f graus: alvo -> %.2f\n", valor,
               ultimo_yaw_graus + valor);
        break;
    case 'z':
        enviar_tc(TC_YAW_ALVO_GRAUS, ultimo_yaw_graus);
        printf("# yaw alvo -> yaw atual (%.2f graus)\n", ultimo_yaw_graus);
        break;
    case 'o':
        enviar_tc(TC_OFFSET_VISUAL_GRAUS, valor);
        printf("# offset visual -> %.2f graus\n", valor);
        break;
    case 't':
        telemetria_ligada = !telemetria_ligada;
        printf("# telemetria %s\n", telemetria_ligada ? "ligada" : "desligada");
        break;
    case 'c':
        imprimir_cabecalho();
        break;
    case 'i':
        imprimir_estado();
        break;
    case 'R':
        printf("# a reiniciar\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        break;
    case '?':
    case 'h':
        imprimir_ajuda();
        break;
    default:
        printf("# comando desconhecido: '%c' (? para ajuda)\n", comando);
        break;
    }
    fflush(stdout);
}

static void tarefa_usb(void *arg) {
    (void)arg;
    char linha[USB_LINHA_MAX];
    int n = 0;
    TelemetriaControlo tm;

    vTaskDelay(pdMS_TO_TICKS(300));   /* da tempo ao host de abrir a porta */
    imprimir_ajuda();
    imprimir_cabecalho();

    for (;;) {
        uint8_t entrada[32];
        int lidos = usb_serial_jtag_read_bytes(entrada, sizeof(entrada),
                                               pdMS_TO_TICKS(5));
        for (int i = 0; i < lidos; ++i) {
            char c = (char)entrada[i];
            if (c == '\n' || c == '\r') {
                if (n > 0) {
                    linha[n] = '\0';
                    processar_linha(linha);
                    n = 0;
                }
            } else if (n < USB_LINHA_MAX - 1) {
                linha[n++] = c;
            } else {
                n = 0;   /* linha demasiado longa: descartar */
            }
        }

        /* A fila de telemetria tem tamanho 1 com xQueueOverwrite: ler a 200 Hz
         * garante que nao se perde nenhuma das 20 amostras por segundo. */
        while (app_controlo_ler_telemetria(&tm)) {
            if (telemetria_ligada) {
                imprimir_telemetria(&tm);
            }
        }
    }
}

int app_usb_iniciar(void) {
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = USB_TX_BUFFER,
        .rx_buffer_size = USB_RX_BUFFER,
    };
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "nao consegui instalar o driver USB Serial JTAG");
        return 0;
    }
    USB_LIGAR_VFS();
    setvbuf(stdout, NULL, _IOLBF, 512);

    /* Prioridade baixa e no Core 0: nunca rouba tempo ao controlo (Core 1). */
    if (xTaskCreatePinnedToCore(tarefa_usb, "usb", 4096, NULL, 5, NULL, 0)
        != pdPASS) {
        ESP_LOGE(TAG, "nao consegui criar a tarefa da consola");
        return 0;
    }
    return 1;
}
