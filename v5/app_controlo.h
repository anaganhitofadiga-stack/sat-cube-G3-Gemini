#ifndef APP_CONTROLO_H
#define APP_CONTROLO_H

/*
 * Integracao FreeRTOS (ESP-IDF, ESP32-S3) — V5
 *  - Tarefa de controlo fixa no Core 1, a 200 Hz, prioridade alta.
 *  - A camara (Core 0) entrega medidas com app_controlo_publicar_medida_visual().
 *  - A Ground Station (Core 0) envia telecomandos e le telemetria por filas.
 */

#include <stdint.h>
#include "controlo_visual.h"

/* Calibracao da camara (preencher com os valores medidos pelo Ricardo).
 * f_px = (largura_px / 2) / tan(FOV_horizontal / 2).
 * Ex.: QVGA 320 px e FOV horizontal de 60 graus -> f ~ 277 px. */
#define CAMERA_DISTANCIA_FOCAL_PX   277.0f
#define CAMERA_SINAL                (+1)   /* trocar para -1 se o cubo fugir do alvo */

#define APP_PRIORIDADE_CONTROLO     20     /* acima da camara e do Wi-Fi */
#define APP_CORE_CONTROLO           1
#define APP_STACK_CONTROLO          6144
#define APP_DECIMACAO_TELEMETRIA    10     /* 200 Hz / 10 = 20 Hz para a GS */
#define APP_MAX_FALHAS_IMU          10     /* 10 ciclos = 50 ms sem IMU -> para o motor */
#define APP_MAX_FALHAS_MOTEUS       20     /* 100 ms sem resposta -> falha */
#define APP_AMOSTRAS_CALIB_GIRO     400    /* 2 s parado para medir o vies */
#define APP_LIMIAR_MOVIMENTO_CALIB  0.05f  /* rad/s: se mexer, repete a calibracao */
#define APP_VEL_RODA_DISPARO_RAD_S  376.99f /* 3600 rpm: paragem de emergencia */

typedef enum {
    MODO_PARADO = 0,    /* motor sem corrente */
    MODO_ATITUDE_GS,    /* nivel 3: angulo pedido pela Ground Station */
    MODO_VISUAL         /* niveis 5-6: angulo vindo da camara */
} ModoControlo;

typedef enum {
    TC_ARMAR = 0,
    TC_DESARMAR,
    TC_PARAGEM_EMERGENCIA,
    TC_MODO,            /* valor = ModoControlo */
    TC_YAW_ALVO_GRAUS,  /* valor = angulo absoluto */
    TC_OFFSET_VISUAL_GRAUS
} TipoTelecomando;

typedef struct {
    uint8_t tipo;       /* TipoTelecomando */
    float valor;
} TelecomandoControlo;

typedef enum {
    FALHA_NENHUMA = 0,
    FALHA_IMU = 1,
    FALHA_MOTEUS = 2,
    FALHA_SOBREVELOCIDADE = 4,
    FALHA_CICLO_ATRASADO = 8,
    FALHA_CALIBRACAO = 16,
    FALHA_EMERGENCIA = 32
} CodigoFalha;

typedef struct {
    int64_t t_us;
    float roll, pitch, yaw;          /* rad */
    float wx, wy, wz;                /* rad/s */
    float yaw_alvo;                  /* rad */
    float vel_corpo_desejada;        /* rad/s */
    float torque_roda;               /* N.m */
    float vel_roda;                  /* rad/s */
    float dt;                        /* s */
    uint8_t modo;                    /* ModoControlo */
    uint8_t estado_visual;           /* EstadoVisual */
    uint8_t armado;
    uint8_t limitado;
    uint32_t falhas;                 /* mascara CodigoFalha */
    uint32_t imagens_aceites;
    uint32_t imagens_rejeitadas;
} TelemetriaControlo;

/* Cria filas, temporizador e tarefa. Chamar uma vez a partir do app_main(). */
int app_controlo_iniciar(void);

/* Tarefa da camara: entrega a medida mais recente (sobrescreve a anterior). */
void app_controlo_publicar_medida_visual(const MedidaVisual *m);

/* Tarefa TMTC: envia um telecomando (nao bloqueia). Devolve 1 se aceite. */
int app_controlo_enviar_telecomando(const TelecomandoControlo *tc);

/* Tarefa TMTC: le a telemetria mais recente. Devolve 1 se havia dados novos. */
int app_controlo_ler_telemetria(TelemetriaControlo *t);

#endif
