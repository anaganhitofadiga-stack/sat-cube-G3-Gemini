#ifndef CONTROLO_VISUAL_H
#define CONTROLO_VISUAL_H

/*
 * Controlo visual (WBS 4.3 / Niveis 4-5) — V5
 * Converte o erro do alvo na imagem (pixeis ou graus) num alvo de yaw
 * absoluto, compensando o atraso da camara com o historico do yaw da IMU.
 * Todas as funcoes sao chamadas APENAS pela tarefa de controlo (Core 1);
 * a tarefa da camara so entrega MedidaVisual atraves de uma fila FreeRTOS.
 */

#include <stdint.h>

/* Imagem com idade superior a isto e descartada (latencia esperada ~0,1 s). */
#define VISUAL_IDADE_MAX_S           0.25f
/* Sem imagem valida durante mais do que isto -> estado OCLUSAO. */
#define VISUAL_TIMEOUT_OCLUSAO_S     0.3f
/* Sem imagem valida durante mais do que isto -> estado SEM_ALVO. */
#define VISUAL_TIMEOUT_PERDIDO_S     5.0f
/* Saltos do alvo maiores do que isto (15 graus) precisam de confirmacao. */
#define VISUAL_SALTO_MAX_RAD         0.2618f
#define VISUAL_CONFIRMACOES          3
#define VISUAL_TOLERANCIA_CONF_RAD   0.0524f   /* 3 graus */

/* Historico do yaw: 128 amostras x 5 ms = 0,64 s (> VISUAL_IDADE_MAX_S). */
#define HISTORICO_YAW_TAMANHO        128
#define HISTORICO_FUTURO_MAX_US      10000     /* tolera 10 ms de desvio de relogio */

typedef enum {
    VISUAL_UNIDADE_PIXEIS = 0,
    VISUAL_UNIDADE_GRAUS = 1
} UnidadeVisual;

/* Mensagem enviada pela tarefa da camara (Core 0) para a de controlo (Core 1). */
typedef struct {
    int64_t t_captura_us;     /* instante da CAPTURA (mesmo relogio que esp_timer_get_time) */
    float erro_horizontal;    /* alvo - centro da imagem, em pixeis ou graus */
    uint8_t unidade;          /* UnidadeVisual */
    uint8_t alvo_detetado;    /* 0 se nao encontrou o ponto nesta imagem */
} MedidaVisual;

typedef struct {
    int64_t t_us[HISTORICO_YAW_TAMANHO];
    float yaw[HISTORICO_YAW_TAMANHO];
    int proximo;
    int contagem;
} HistoricoYaw;

typedef enum {
    VISUAL_SEM_ALVO = 0,  /* ainda nao viu o alvo (ou perdeu-o): segura o yaw */
    VISUAL_A_SEGUIR = 1,  /* imagens validas a chegar */
    VISUAL_OCLUSAO = 2    /* imagens deixaram de chegar ha pouco: segura o ultimo alvo com a IMU */
} EstadoVisual;

typedef struct {
    float distancia_focal_px;
    int sinal_camera;
    float offset_alvo_rad;      /* 0 = centrar; !=0 = ponto fora do centro (nivel 7) */
    EstadoVisual estado;
    float yaw_alvo;
    int alvo_definido;
    int64_t t_ultima_valida_us;
    int confirmacoes;
    float yaw_candidato;
    uint32_t imagens_aceites;
    uint32_t imagens_rejeitadas;
} RastreadorVisual;

/* Converte erro (pixeis ou graus) em radianos. Devolve 1 se valido. */
int converter_erro_visual_rad(float erro, UnidadeVisual unidade,
                              float distancia_focal_px, int sinal_camera,
                              float *erro_rad);

/*
 * Funcao da V4, mantida por compatibilidade: erro em pixeis + yaw no instante
 * da captura -> yaw alvo. Rejeita imagens com idade > VISUAL_IDADE_MAX_S.
 */
int atualizar_alvo_yaw_visual(float erro_pixeis,
                              float distancia_focal_px,
                              float yaw_captura_rad,
                              int sinal_camera,
                              float idade_imagem_s,
                              float *yaw_alvo_rad);

void historico_yaw_init(HistoricoYaw *h);
/* Regista o yaw medido no instante t_us (tempos tem de ser crescentes). */
void historico_yaw_registar(HistoricoYaw *h, int64_t t_us, float yaw);
/* Yaw interpolado no instante t_us. Devolve 0 se t_us estiver fora do historico. */
int historico_yaw_obter(const HistoricoYaw *h, int64_t t_us, float *yaw);

int rastreador_init(RastreadorVisual *r, float distancia_focal_px,
                    int sinal_camera);
/* Esquece o alvo: na proxima chamada a rastreador_obter_alvo segura o yaw atual. */
void rastreador_reiniciar(RastreadorVisual *r);
void rastreador_definir_offset(RastreadorVisual *r, float offset_rad);
/* Processa uma imagem. Devolve 1 se o alvo de yaw foi atualizado. */
int rastreador_processar_medida(RastreadorVisual *r, const HistoricoYaw *h,
                                const MedidaVisual *m, int64_t agora_us);
/* Atualiza o estado (timeouts) e devolve o yaw alvo a usar neste ciclo. */
EstadoVisual rastreador_obter_alvo(RastreadorVisual *r, int64_t agora_us,
                                   float yaw_atual, float *yaw_alvo);

#endif
