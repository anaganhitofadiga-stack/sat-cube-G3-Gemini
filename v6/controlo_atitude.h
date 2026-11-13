#ifndef CONTROLO_ATITUDE_H
#define CONTROLO_ATITUDE_H

/*
 * Controlo de atitude do cubo — V5
 * Unidades: rad, rad/s, s, N.m. Ver CHANGELOG.md para as diferencas face a V4.
 */

/* ---------------- Limites do atuador (Moteus r4.11 + mj5208) ---------------- */
#define MAX_TORQUE_COMMAND        0.2f     /* N.m — limite de seguranca (pico do motor ~1,7 N.m) */
#define MAX_TARGET_VELOCITY       1.0f     /* rad/s — velocidade maxima pedida ao corpo (~57 graus/s) */
#define MAX_WHEEL_SPEED_RAD_S     314.16f  /* rad/s — 3000 rpm: a roda nunca e acelerada para la disto */
#define SOFT_WHEEL_SPEED_RAD_S    251.33f  /* rad/s — 2400 rpm: inicio da reducao progressiva do binario */

/* ---------------- Tempo de amostragem ---------------- */
#define CONTROL_PERIOD_S          0.005f   /* 200 Hz na tarefa de controlo (Core 1) */
#define DT_MIN_S                  0.0005f  /* dt menor: ciclo ignorado (protege contra divisao por ~0) */
#define DT_MAX_S                  0.05f    /* dt maior: dt limitado e derivada reiniciada */

/* ---------------- Ganhos iniciais (afinar no hardware) ---------------- */
/* Malha externa: erro de angulo yaw (rad) -> velocidade desejada do corpo (rad/s) */
#define KP_POS_YAW                2.0f
#define KI_POS_YAW                0.05f
#define KD_POS_YAW                0.1f
#define IMAX_POS_YAW              5.0f     /* limite do integral (rad.s) */
#define TAU_D_POS_YAW             0.02f    /* filtro da derivada (s) */

/* Malha interna: erro de velocidade (rad/s) -> binario no corpo (N.m) */
#define KP_VEL_YAW                0.5f
#define KI_VEL_YAW                0.1f
#define KD_VEL_YAW                0.0f
#define IMAX_VEL_YAW              0.2f     /* limite do integral (rad) */
#define TAU_D_VEL_YAW             0.0f

#define GANHO_DAMPING_ROLL        0.05f
#define GANHO_DAMPING_PITCH       0.05f

/* ---------------- Filtro da IMU ---------------- */
/* Atraso do filtro soma-se ao atraso da malha: confirmar com o ruido real. */
#define IMU_GYRO_FILTER_TAU       0.05f    /* s */
/* Constante de tempo do filtro complementar. 0,49 s equivale a alpha = 0,98 a 100 Hz,
 * mas agora o comportamento e o mesmo a qualquer frequencia de amostragem. */
#define IMU_COMPLEMENTARY_TAU     0.49f    /* s */
#define IMU_GRAVITY               9.80665f /* m/s^2 */
#define IMU_DT_MAX_S              0.25f    /* amostra mais espacada do que isto e rejeitada */

typedef struct {
    /* Ganhos e configuracao */
    float kp;
    float ki;
    float kd;
    float i_max;               /* limite simetrico do integral (anti-windup por clamping) */
    float tau_derivada;        /* filtro passa-baixo da derivada (s); 0 = sem filtro */
    int derivada_na_medida;    /* 1: derivada da medida (sem pico quando o alvo salta) */
    int medida_angular;        /* 1: variacao da medida envolvida em [-pi, pi] */
    /* Estado interno */
    float integral;
    float erro_anterior;
    float medida_anterior;
    float derivada_filtrada;
    int primeira_amostra;      /* 1 ate ao primeiro calculo: derivada = 0 */
    int saturado;              /* 1 se a ultima saida foi cortada pelos limites */
} PID_Controller;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude3;

typedef struct {
    Attitude3 atitude;
    Attitude3 velocidade_angular;
} AttitudeEstimate;

typedef struct {
    /* Referencia do corpo (rad/s), nao velocidade fisica do volante. */
    float velocidade_angular_corpo_desejada;
    /* Binario pedido ao volante (N.m). Positivo acelera o volante no sentido positivo. */
    float torque_roda;
    /* 1 se o binario foi cortado (limite de binario ou de velocidade da roda). */
    int limitado;
} WheelCommand;

typedef struct {
    WheelCommand roll;
    WheelCommand pitch;
    WheelCommand yaw;
} WheelCommands;

typedef struct {
    AttitudeEstimate estimativa;
    int inicializado;
} IMU_Filter;

/* ---------------- Utilitarios ---------------- */
/* Envolve qualquer angulo finito para [-pi, pi]. */
float envolver_angulo(float angulo);

/* ---------------- Controladores ---------------- */
/* Devolve 1 se os ganhos foram aceites, 0 caso contrario. */
int init_controladores(void);
/* Limpa integrais, memorias de derivada e flags de saturacao (ganhos ficam). */
void reset_controladores(void);

int inicializar_pid(PID_Controller *pid, float kp, float ki, float kd,
                    float i_max);
int configurar_derivada_pid(PID_Controller *pid, int derivada_na_medida,
                            int medida_angular, float tau_derivada);
void reiniciar_pid(PID_Controller *pid);

/*
 * erro = referencia - medida. 'medida' so e usada quando derivada_na_medida = 1.
 * congelar_integral = 1 impede o integral de crescer (anti-windup em cascata),
 * mas deixa-o diminuir.
 */
float calcular_pid(PID_Controller *pid, float erro, float medida, float dt,
                   float saida_minima, float saida_maxima,
                   int congelar_integral);

float calcular_malha_externa_yaw(float erro_posicao, float yaw_medido,
                                 float dt, int congelar_integral);
float calcular_malha_intermedia_yaw(float vel_angular_desejada,
                                    float vel_angular_real, float dt,
                                    int congelar_integral);
float calcular_amortecimento_roll(float vel_angular_real_roll);
float calcular_amortecimento_pitch(float vel_angular_real_pitch);

/*
 * Protecao do atuador: limita o binario a MAX_TORQUE_COMMAND e, quando a roda
 * se aproxima de MAX_WHEEL_SPEED_RAD_S, reduz progressivamente o binario que a
 * aceleraria ainda mais (o binario que a trava passa sempre). vel_roda_rad_s
 * pode ser NAN se a velocidade for desconhecida (so limita o binario).
 */
float proteger_saturacao_roda(float torque_roda, float vel_roda_rad_s,
                              int *limitado);

/*
 * Calcula os comandos dos tres canais. vel_rodas (rad/s) pode ser NULL ou ter
 * campos NAN. Angulos yaw sao envolvidos automaticamente para [-pi, pi].
 * Devolve 0 (e comandos a zero) com entradas invalidas ou dt < DT_MIN_S.
 */
int calcular_comandos_rodas(const AttitudeEstimate *estado,
                            const Attitude3 *atitude_alvo,
                            const Attitude3 *vel_rodas,
                            float dt,
                            WheelCommands *comandos);

/* ---------------- Filtro da IMU ---------------- */
/*
 * imu_filter_update espera aceleracao em m/s^2, giroscopio em rad/s (ja sem
 * vies) e magnetometro calibrado; dt em segundos. Magnetometro opcional:
 * passar mx = my = mz = 0 para usar so o giroscopio no yaw (a camara corrige
 * a deriva). Devolve 1 com amostra valida, 0 caso contrario.
 * Limitacao: integra gx/gy/gz como taxas de Euler (valido para pequenas
 * inclinacoes de roll/pitch).
 */
void imu_filter_init(IMU_Filter *filtro);
int imu_filter_update(IMU_Filter *filtro,
                      float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      float dt);

#endif
