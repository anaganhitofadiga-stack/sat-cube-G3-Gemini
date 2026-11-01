#ifndef CONTROLO_ATITUDE_H
#define CONTROLO_ATITUDE_H

/*
 * Limites e ganhos do controlador.
 *
 * Os valores são parâmetros iniciais para simulação/afinação: não substituem
 * os limites certificados do motor, da roda, do Moteus ou da estrutura.
 */
#define MAX_TORQUE_COMMAND   0.2f   // Binário máximo comandado à roda (N.m).
#define MAX_TARGET_VELOCITY  1.0f   // Velocidade angular máxima desejada em yaw (rad/s).

/* Ganhos PID da malha externa: erro de posição (rad) -> velocidade desejada (rad/s). */
#define KP_POS_YAW           2.0f
#define KI_POS_YAW           0.05f
#define KD_POS_YAW           0.1f
#define IMAX_POS_YAW         5.0f    // Limite do acumulador integral (rad.s).

/* Ganhos da malha interna: erro de velocidade (rad/s) -> binário de roda (N.m). */
#define KP_VEL_YAW           0.5f
#define KI_VEL_YAW           0.1f
#define KD_VEL_YAW           0.0f    // Sem ação derivativa; a malha interna funciona como PI.
#define IMAX_VEL_YAW         1.0f    // Limite do acumulador integral ((rad/s).s).

/* Ganhos proporcionais de amortecimento; multiplicados pela velocidade angular (rad/s). */
#define GANHO_DAMPING_ROLL   0.05f
#define GANHO_DAMPING_PITCH  0.05f

/*
 * Estado de um controlador PID.
 * Os dois controladores yaw guardam uma instância desta estrutura cada.
 */
typedef struct {
    float kp;               // Ganho proporcional: resposta ao erro atual.
    float ki;               // Ganho integral: resposta ao erro acumulado no tempo.
    float kd;               // Ganho derivativo: resposta à variação do erro.
    float integral;         // Soma temporal do erro, limitada por i_max.
    float erro_anterior;    // Erro do ciclo anterior, usado para calcular a derivada.
    float i_max;            // Valor máximo absoluto permitido para integral.
} PID_Controller;

/* Carrega os ganhos definidos acima e prepara o estado inicial dos controladores. */
void init_controladores(void);

/* Limpa os acumuladores e erros anteriores, mantendo os ganhos configurados. */
void reset_controladores(void);

/*
 * Calcula a velocidade angular desejada para yaw.
 * Entrada: erro angular em radianos e intervalo desde o ciclo anterior em segundos.
 * Saída: velocidade angular desejada em rad/s, limitada por MAX_TARGET_VELOCITY.
 */
float calcular_malha_externa_yaw(float erro_posicao, float dt);

/*
 * Calcula o binário de comando da roda de yaw a partir da velocidade desejada
 * e da velocidade medida pela IMU. As velocidades usam rad/s, dt usa segundos
 * e a saída usa N.m.
 */
float calcular_malha_intermedia_yaw(float vel_angular_desejada, float vel_angular_real, float dt);

/*
 * Calcula o binário de comando da roda para amortecer roll/pitch.
 * A velocidade de entrada é em rad/s e o binário de saída em N.m.
 */
float calcular_amortecimento_roll(float vel_angular_real_roll);
float calcular_amortecimento_pitch(float vel_angular_real_pitch);

#endif /* CONTROLO_ATITUDE_H */