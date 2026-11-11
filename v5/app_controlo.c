/*
 * app_controlo.c — Integracao FreeRTOS (ESP-IDF) do controlo de atitude, V5.
 * Este ficheiro so compila no ESP-IDF (nao entra no teste em MSYS2).
 *
 * Fluxo de cada ciclo (200 Hz, Core 1):
 *   1. espera o "tique" do temporizador (esp_timer, 5 ms)
 *   2. mede o dt real
 *   3. aplica telecomandos da Ground Station
 *   4. le a IMU (+ magnetometro opcional) e atualiza o filtro
 *   5. guarda o yaw no historico (para compensar o atraso da camara)
 *   6. se chegou imagem nova, atualiza o alvo visual
 *   7. calcula o binario (cascata PID + protecoes) e envia ao moteus
 *   8. publica telemetria a 20 Hz
 */

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "app_controlo.h"
#include "controlo_atitude.h"
#include "controlo_visual.h"
#include "hal_cubo.h"

#define USAR_MAGNETOMETRO 0   /* 1 so depois de calibrar o LIS3MDL longe do motor */

static const char *TAG = "controlo";

static TaskHandle_t h_tarefa_controlo;
static QueueHandle_t fila_visual;       /* tamanho 1, xQueueOverwrite */
static QueueHandle_t fila_telecomandos; /* tamanho 8 */
static QueueHandle_t fila_telemetria;   /* tamanho 1, xQueueOverwrite */
static esp_timer_handle_t temporizador_controlo;

/* Chamado pelo esp_timer a cada 5 ms (corre na tarefa do esp_timer, nao numa ISR). */
static void tique_controlo(void *arg) {
    (void)arg;
    if (h_tarefa_controlo != NULL) {
        xTaskNotifyGive(h_tarefa_controlo);
    }
}

void app_controlo_publicar_medida_visual(const MedidaVisual *m) {
    if (fila_visual != NULL && m != NULL) {
        xQueueOverwrite(fila_visual, m);  /* so interessa a imagem mais recente */
    }
}

int app_controlo_enviar_telecomando(const TelecomandoControlo *tc) {
    if (fila_telecomandos == NULL || tc == NULL) {
        return 0;
    }
    /* A paragem de emergencia nunca pode ficar de fora por fila cheia. */
    if (tc->tipo == TC_PARAGEM_EMERGENCIA) {
        xQueueReset(fila_telecomandos);
    }
    return xQueueSend(fila_telecomandos, tc, 0) == pdTRUE;
}

int app_controlo_ler_telemetria(TelemetriaControlo *t) {
    if (fila_telemetria == NULL || t == NULL) {
        return 0;
    }
    return xQueueReceive(fila_telemetria, t, 0) == pdTRUE;
}

/* Mede o vies do giroscopio com o cubo parado (motor sem corrente). */
static int calibrar_giroscopio(float vies[3]) {
    for (int tentativa = 0; tentativa < 5; ++tentativa) {
        double soma[3] = {0.0, 0.0, 0.0};
        float minimo[3] = {INFINITY, INFINITY, INFINITY};
        float maximo[3] = {-INFINITY, -INFINITY, -INFINITY};
        int validas = 0;
        for (int i = 0; i < APP_AMOSTRAS_CALIB_GIRO; ++i) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
            AmostraIMU a;
            if (!hal_imu_ler(&a)) {
                continue;
            }
            float g[3] = {a.gx, a.gy, a.gz};
            for (int k = 0; k < 3; ++k) {
                soma[k] += g[k];
                if (g[k] < minimo[k]) minimo[k] = g[k];
                if (g[k] > maximo[k]) maximo[k] = g[k];
            }
            validas++;
            float v_parado = 0.0f;
            hal_moteus_parar(&v_parado);  /* mantem o moteus seguro e o watchdog alimentado */
        }
        int parado = validas > APP_AMOSTRAS_CALIB_GIRO / 2;
        for (int k = 0; k < 3 && parado; ++k) {
            if (maximo[k] - minimo[k] > 2.0f * APP_LIMIAR_MOVIMENTO_CALIB) {
                parado = 0;
            }
        }
        if (parado) {
            for (int k = 0; k < 3; ++k) {
                vies[k] = (float)(soma[k] / validas);
            }
            ESP_LOGI(TAG, "vies giro: %.4f %.4f %.4f rad/s",
                     vies[0], vies[1], vies[2]);
            return 1;
        }
        ESP_LOGW(TAG, "cubo em movimento durante a calibracao, a repetir");
    }
    return 0;
}

static void tarefa_controlo(void *arg) {
    (void)arg;
    IMU_Filter filtro;
    HistoricoYaw historico;
    RastreadorVisual rastreador;
    WheelCommands comandos;
    TelemetriaControlo tm;
    float vies_giro[3] = {0.0f, 0.0f, 0.0f};

    memset(&comandos, 0, sizeof(comandos));
    memset(&tm, 0, sizeof(tm));
    imu_filter_init(&filtro);
    historico_yaw_init(&historico);
    rastreador_init(&rastreador, CAMERA_DISTANCIA_FOCAL_PX, CAMERA_SINAL);

    uint32_t falhas = FALHA_NENHUMA;
    if (!init_controladores()) {
        ESP_LOGE(TAG, "ganhos invalidos");
        falhas |= FALHA_CALIBRACAO;
    }
    if (!calibrar_giroscopio(vies_giro)) {
        ESP_LOGE(TAG, "calibracao do giroscopio falhou");
        falhas |= FALHA_CALIBRACAO;
    }

    ModoControlo modo = MODO_PARADO;
    int armado = 0;
    float yaw_alvo_gs = 0.0f;
    float vel_roda = NAN;
    float torque_aplicado = 0.0f;
    int falhas_imu = 0;
    int falhas_moteus = 0;
    uint32_t ciclo = 0;
    int64_t t_anterior = hal_tempo_us();

    for (;;) {
        /* 1. Espera o tique. Se nao chegar em 20 ms, algo bloqueou o sistema. */
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
            hal_moteus_parar(&vel_roda);
            falhas |= FALHA_CICLO_ATRASADO;
            armado = 0;
            t_anterior = hal_tempo_us();
            continue;
        }

        /* 2. dt real (o temporizador tem jitter; o controlo usa o valor medido). */
        int64_t agora = hal_tempo_us();
        float dt = (float)(agora - t_anterior) * 1.0e-6f;
        t_anterior = agora;

        /* 3. Telecomandos. */
        TelecomandoControlo tc;
        while (xQueueReceive(fila_telecomandos, &tc, 0) == pdTRUE) {
            switch (tc.tipo) {
            case TC_ARMAR:
                if ((falhas & (FALHA_CALIBRACAO | FALHA_EMERGENCIA)) == 0) {
                    falhas &= ~(uint32_t)(FALHA_IMU | FALHA_MOTEUS |
                                          FALHA_CICLO_ATRASADO |
                                          FALHA_SOBREVELOCIDADE);
                    reset_controladores();
                    armado = 1;
                }
                break;
            case TC_DESARMAR:
                armado = 0;
                break;
            case TC_PARAGEM_EMERGENCIA:
                armado = 0;
                falhas |= FALHA_EMERGENCIA;   /* so sai com reinicio da placa */
                break;
            case TC_MODO:
                if (tc.valor >= MODO_PARADO && tc.valor <= MODO_VISUAL) {
                    ModoControlo novo = (ModoControlo)(int)tc.valor;
                    if (novo != modo) {
                        reset_controladores();       /* troca sem "solavancos" */
                        if (novo == MODO_VISUAL &&
                            rastreador.estado != VISUAL_A_SEGUIR) {
                            rastreador_reiniciar(&rastreador); /* segura o yaw atual */
                        }
                        if (novo == MODO_ATITUDE_GS) {
                            yaw_alvo_gs = filtro.estimativa.atitude.yaw;
                        }
                        modo = novo;
                    }
                }
                break;
            case TC_YAW_ALVO_GRAUS:
                if (isfinite(tc.valor)) {
                    yaw_alvo_gs = envolver_angulo(tc.valor * 0.01745329252f);
                }
                break;
            case TC_OFFSET_VISUAL_GRAUS:
                rastreador_definir_offset(&rastreador, tc.valor * 0.01745329252f);
                break;
            default:
                break;
            }
        }

        /* 4. IMU. */
        AmostraIMU a;
        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        int imu_ok = hal_imu_ler(&a);
#if USAR_MAGNETOMETRO
        if (imu_ok && !hal_mag_ler(&mx, &my, &mz)) {
            mx = my = mz = 0.0f;  /* sem magnetometro: so giroscopio + camara */
        }
#endif
        imu_ok = imu_ok && imu_filter_update(&filtro,
                                             a.ax, a.ay, a.az,
                                             a.gx - vies_giro[0],
                                             a.gy - vies_giro[1],
                                             a.gz - vies_giro[2],
                                             mx, my, mz, dt);
        if (!imu_ok) {
            if (++falhas_imu >= APP_MAX_FALHAS_IMU) {
                falhas |= FALHA_IMU;
                armado = 0;
                hal_moteus_parar(&vel_roda);
            } else if (armado) {
                /* Falha pontual: repete o ultimo binario (alimenta o watchdog). */
                hal_moteus_comandar_binario(torque_aplicado, &vel_roda);
            }
            continue;
        }
        falhas_imu = 0;
        const AttitudeEstimate *est = &filtro.estimativa;

        /* 5. Historico do yaw para casar cada imagem com o yaw da sua captura. */
        historico_yaw_registar(&historico, agora, est->atitude.yaw);

        /* 6. Camara: so a medida mais recente (fila de tamanho 1). */
        MedidaVisual medida;
        if (xQueueReceive(fila_visual, &medida, 0) == pdTRUE) {
            rastreador_processar_medida(&rastreador, &historico, &medida, agora);
        }
        float yaw_alvo_visual = est->atitude.yaw;
        EstadoVisual estado_visual = rastreador_obter_alvo(
            &rastreador, agora, est->atitude.yaw, &yaw_alvo_visual);

        /* 7. Controlo + atuacao. */
        Attitude3 alvo = {0.0f, 0.0f,
                          (modo == MODO_VISUAL) ? yaw_alvo_visual : yaw_alvo_gs};
        Attitude3 vel_rodas = {NAN, NAN, vel_roda};
        torque_aplicado = 0.0f;

        if (isfinite(vel_roda) && fabsf(vel_roda) > APP_VEL_RODA_DISPARO_RAD_S) {
            falhas |= FALHA_SOBREVELOCIDADE;
            armado = 0;
        }

        int ok_moteus;
        if (armado && modo != MODO_PARADO && falhas == FALHA_NENHUMA) {
            if (calcular_comandos_rodas(est, &alvo, &vel_rodas, dt, &comandos)) {
                torque_aplicado = comandos.yaw.torque_roda;
            }
            ok_moteus = hal_moteus_comandar_binario(torque_aplicado, &vel_roda);
        } else {
            reset_controladores();
            memset(&comandos, 0, sizeof(comandos));
            ok_moteus = hal_moteus_parar(&vel_roda);
        }
        if (!ok_moteus) {
            vel_roda = NAN;   /* velocidade desconhecida: so o limite de binario atua */
            if (++falhas_moteus >= APP_MAX_FALHAS_MOTEUS) {
                falhas |= FALHA_MOTEUS;
                armado = 0;
            }
        } else {
            falhas_moteus = 0;
        }

        /* 8. Telemetria (20 Hz). */
        if (++ciclo % APP_DECIMACAO_TELEMETRIA == 0) {
            tm.t_us = agora;
            tm.roll = est->atitude.roll;
            tm.pitch = est->atitude.pitch;
            tm.yaw = est->atitude.yaw;
            tm.wx = est->velocidade_angular.roll;
            tm.wy = est->velocidade_angular.pitch;
            tm.wz = est->velocidade_angular.yaw;
            tm.yaw_alvo = alvo.yaw;
            tm.vel_corpo_desejada = comandos.yaw.velocidade_angular_corpo_desejada;
            tm.torque_roda = torque_aplicado;
            tm.vel_roda = vel_roda;
            tm.dt = dt;
            tm.modo = (uint8_t)modo;
            tm.estado_visual = (uint8_t)estado_visual;
            tm.armado = (uint8_t)armado;
            tm.limitado = (uint8_t)comandos.yaw.limitado;
            tm.falhas = falhas;
            tm.imagens_aceites = rastreador.imagens_aceites;
            tm.imagens_rejeitadas = rastreador.imagens_rejeitadas;
            xQueueOverwrite(fila_telemetria, &tm);
        }
    }
}

int app_controlo_iniciar(void) {
    fila_visual = xQueueCreate(1, sizeof(MedidaVisual));
    fila_telecomandos = xQueueCreate(8, sizeof(TelecomandoControlo));
    fila_telemetria = xQueueCreate(1, sizeof(TelemetriaControlo));
    if (fila_visual == NULL || fila_telecomandos == NULL ||
        fila_telemetria == NULL) {
        return 0;
    }

    if (xTaskCreatePinnedToCore(tarefa_controlo, "controlo",
                                APP_STACK_CONTROLO, NULL,
                                APP_PRIORIDADE_CONTROLO,
                                &h_tarefa_controlo,
                                APP_CORE_CONTROLO) != pdPASS) {
        return 0;
    }

    const esp_timer_create_args_t args = {
        .callback = &tique_controlo,
        .arg = NULL,
        .name = "tique_controlo"
    };
    if (esp_timer_create(&args, &temporizador_controlo) != ESP_OK ||
        esp_timer_start_periodic(temporizador_controlo,
                                 (uint64_t)(CONTROL_PERIOD_S * 1.0e6f)) != ESP_OK) {
        return 0;
    }
    return 1;
}

/*
 * ---------------------------------------------------------------------------
 * Exemplo para a tarefa da camara (Ricardo, Core 0) — NAO compilado aqui:
 *
 *   camera_fb_t *fb = esp_camera_fb_get();
 *   int64_t t_cap = (int64_t)fb->timestamp.tv_sec * 1000000LL +
 *                   fb->timestamp.tv_usec;   // confirmar que usa o esp_timer
 *   float cx;                                 // coluna do centro do ponto branco
 *   int encontrou = detetar_ponto_branco(fb, &cx);
 *   MedidaVisual m = {
 *       .t_captura_us = t_cap,
 *       .erro_horizontal = encontrou ? (cx - fb->width / 2.0f) : 0.0f,
 *       .unidade = VISUAL_UNIDADE_PIXEIS,
 *       .alvo_detetado = (uint8_t)encontrou
 *   };
 *   esp_camera_fb_return(fb);
 *   app_controlo_publicar_medida_visual(&m);
 * ---------------------------------------------------------------------------
 */
