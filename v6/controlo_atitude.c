#include "controlo_atitude.h"
#include <math.h>

#define PI_F 3.14159265358979323846f
#define DOIS_PI_F 6.28318530717958647692f
#define ACCEL_MIN_G 0.7f
#define ACCEL_MAX_G 1.3f
#define MAG_NORMA_MIN 1.0e-6f

static PID_Controller pid_posicao_yaw;
static PID_Controller pid_velocidade_yaw;
static float ganho_damping_roll;
static float ganho_damping_pitch;
/* Memoria de saturacao do ciclo anterior (anti-windup em cascata). */
static int roda_yaw_limitada_anterior;

static float limitar(float valor, float minimo, float maximo) {
    if (valor > maximo) {
        return maximo;
    }
    if (valor < minimo) {
        return minimo;
    }
    return valor;
}

/*
 * Valida e prepara o dt dos controladores.
 * Devolve 0 se o dt for invalido (NaN, infinito, negativo ou < DT_MIN_S).
 * Se dt > DT_MAX_S (tarefa atrasada), devolve DT_MAX_S e pede para reiniciar
 * a derivada: assim um atraso nao provoca um salto no integral nem na derivada.
 */
static float preparar_dt(float dt, int *reiniciar_derivada) {
    if (!isfinite(dt) || dt < DT_MIN_S) {
        return 0.0f;
    }
    if (dt > DT_MAX_S) {
        if (reiniciar_derivada != 0) {
            *reiniciar_derivada = 1;
        }
        return DT_MAX_S;
    }
    return dt;
}

float envolver_angulo(float angulo) {
    if (!isfinite(angulo)) {
        return angulo;
    }
    float r = remainderf(angulo, DOIS_PI_F);
    /* Protecao contra arredondamentos de float ligeiramente fora de [-pi, pi]. */
    return limitar(r, -PI_F, PI_F);
}

void reiniciar_pid(PID_Controller *pid) {
    if (pid == 0) {
        return;
    }
    pid->integral = 0.0f;
    pid->erro_anterior = 0.0f;
    pid->medida_anterior = 0.0f;
    pid->derivada_filtrada = 0.0f;
    pid->primeira_amostra = 1;
    pid->saturado = 0;
}

int inicializar_pid(PID_Controller *pid,
                    float kp, float ki, float kd, float i_max) {
    if (pid == 0 || !isfinite(kp) || !isfinite(ki) ||
        !isfinite(kd) || !isfinite(i_max) || i_max < 0.0f) {
        return 0;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->tau_derivada = 0.0f;
    pid->derivada_na_medida = 0;
    pid->medida_angular = 0;
    reiniciar_pid(pid);
    return 1;
}

int configurar_derivada_pid(PID_Controller *pid, int derivada_na_medida,
                            int medida_angular, float tau_derivada) {
    if (pid == 0 || !isfinite(tau_derivada) || tau_derivada < 0.0f) {
        return 0;
    }
    pid->derivada_na_medida = derivada_na_medida ? 1 : 0;
    pid->medida_angular = medida_angular ? 1 : 0;
    pid->tau_derivada = tau_derivada;
    pid->primeira_amostra = 1;
    return 1;
}

int init_controladores(void) {
    int ok = inicializar_pid(&pid_posicao_yaw, KP_POS_YAW, KI_POS_YAW,
                             KD_POS_YAW, IMAX_POS_YAW);
    /* Derivada sobre o yaw medido: um novo alvo vindo da camara nao gera pico. */
    ok = configurar_derivada_pid(&pid_posicao_yaw, 1, 1, TAU_D_POS_YAW) && ok;
    ok = inicializar_pid(&pid_velocidade_yaw, KP_VEL_YAW, KI_VEL_YAW,
                         KD_VEL_YAW, IMAX_VEL_YAW) && ok;
    ok = configurar_derivada_pid(&pid_velocidade_yaw, 1, 0, TAU_D_VEL_YAW) && ok;
    ganho_damping_roll = GANHO_DAMPING_ROLL;
    ganho_damping_pitch = GANHO_DAMPING_PITCH;
    roda_yaw_limitada_anterior = 0;
    return ok;
}

void reset_controladores(void) {
    reiniciar_pid(&pid_posicao_yaw);
    reiniciar_pid(&pid_velocidade_yaw);
    roda_yaw_limitada_anterior = 0;
}

float calcular_pid(PID_Controller *pid, float erro, float medida, float dt,
                   float saida_minima, float saida_maxima,
                   int congelar_integral) {
    if (pid == 0) {
        return 0.0f;
    }
    pid->saturado = 0;
    if (!isfinite(erro) ||
        !isfinite(saida_minima) || !isfinite(saida_maxima) ||
        saida_minima >= saida_maxima || !isfinite(pid->i_max) ||
        pid->i_max < 0.0f || !isfinite(pid->kp) ||
        !isfinite(pid->ki) || !isfinite(pid->kd) ||
        !isfinite(pid->tau_derivada) || pid->tau_derivada < 0.0f ||
        (pid->derivada_na_medida && !isfinite(medida))) {
        return 0.0f;
    }

    int reiniciar = 0;
    float dt_ok = preparar_dt(dt, &reiniciar);
    if (dt_ok <= 0.0f) {
        return 0.0f;
    }
    if (reiniciar) {
        pid->primeira_amostra = 1;
    }

    /* ---- Termo derivativo ---- */
    float derivada = 0.0f;
    if (!pid->primeira_amostra) {
        float derivada_bruta;
        if (pid->derivada_na_medida) {
            float variacao = medida - pid->medida_anterior;
            if (pid->medida_angular) {
                variacao = envolver_angulo(variacao);
            }
            /* erro = ref - medida  =>  d(erro)/dt = -d(medida)/dt com ref parada */
            derivada_bruta = -variacao / dt_ok;
        } else {
            derivada_bruta = (erro - pid->erro_anterior) / dt_ok;
        }
        if (pid->tau_derivada > 0.0f) {
            float a = dt_ok / (pid->tau_derivada + dt_ok);
            derivada = pid->derivada_filtrada +
                       a * (derivada_bruta - pid->derivada_filtrada);
        } else {
            derivada = derivada_bruta;
        }
    }

    /* ---- Termo integral (clamping) + saida ---- */
    float integral_provisoria = limitar(
        pid->integral + erro * dt_ok, -pid->i_max, pid->i_max);
    float saida_bruta = pid->kp * erro +
                        pid->ki * integral_provisoria +
                        pid->kd * derivada;
    if (!isfinite(saida_bruta)) {
        reiniciar_pid(pid);
        return 0.0f;
    }

    /* Anti-windup por integracao condicional: so integra se a saida nao esta
     * saturada, ou se o erro ajuda a sair da saturacao. */
    int pode_integrar =
        (saida_bruta <= saida_maxima && saida_bruta >= saida_minima) ||
        (saida_bruta > saida_maxima && erro < 0.0f) ||
        (saida_bruta < saida_minima && erro > 0.0f);
    /* Anti-windup em cascata: se o andar seguinte esta saturado, o integral
     * pode encolher mas nao pode crescer. */
    if (congelar_integral &&
        fabsf(integral_provisoria) > fabsf(pid->integral)) {
        pode_integrar = 0;
    }

    if (pode_integrar) {
        pid->integral = integral_provisoria;
    } else {
        /* Saida coerente com o integral que realmente ficou guardado. */
        saida_bruta = pid->kp * erro + pid->ki * pid->integral +
                      pid->kd * derivada;
    }

    float saida = limitar(saida_bruta, saida_minima, saida_maxima);
    pid->saturado = (saida != saida_bruta);
    pid->erro_anterior = erro;
    pid->medida_anterior = medida;
    pid->derivada_filtrada = derivada;
    pid->primeira_amostra = 0;
    return saida;
}

float calcular_malha_externa_yaw(float erro_posicao, float yaw_medido,
                                 float dt, int congelar_integral) {
    return calcular_pid(&pid_posicao_yaw, erro_posicao, yaw_medido, dt,
                        -MAX_TARGET_VELOCITY, MAX_TARGET_VELOCITY,
                        congelar_integral);
}

float calcular_malha_intermedia_yaw(float vel_angular_desejada,
                                    float vel_angular_real, float dt,
                                    int congelar_integral) {
    if (!isfinite(vel_angular_desejada) || !isfinite(vel_angular_real)) {
        return 0.0f;
    }
    float erro_velocidade = vel_angular_desejada - vel_angular_real;
    float torque_corpo = calcular_pid(&pid_velocidade_yaw, erro_velocidade,
                                      vel_angular_real, dt,
                                      -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND,
                                      congelar_integral);
    /* Acao-reacao: o corpo recebe o binario simetrico ao aplicado no volante. */
    return -torque_corpo;
}

float calcular_amortecimento_roll(float vel_angular_real_roll) {
    if (!isfinite(vel_angular_real_roll)) {
        return 0.0f;
    }
    return limitar(ganho_damping_roll * vel_angular_real_roll,
                   -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND);
}

float calcular_amortecimento_pitch(float vel_angular_real_pitch) {
    if (!isfinite(vel_angular_real_pitch)) {
        return 0.0f;
    }
    return limitar(ganho_damping_pitch * vel_angular_real_pitch,
                   -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND);
}

float proteger_saturacao_roda(float torque_roda, float vel_roda_rad_s,
                              int *limitado) {
    int corte = 0;
    float t;
    if (!isfinite(torque_roda)) {
        t = 0.0f;
        corte = 1;
    } else {
        t = limitar(torque_roda, -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND);
        if (t != torque_roda) {
            corte = 1;
        }
        if (isfinite(vel_roda_rad_s)) {
            float v = fabsf(vel_roda_rad_s);
            int acelera = (t > 0.0f && vel_roda_rad_s > 0.0f) ||
                          (t < 0.0f && vel_roda_rad_s < 0.0f);
            if (acelera && v > SOFT_WHEEL_SPEED_RAD_S) {
                float fator = (MAX_WHEEL_SPEED_RAD_S - v) /
                              (MAX_WHEEL_SPEED_RAD_S - SOFT_WHEEL_SPEED_RAD_S);
                t *= limitar(fator, 0.0f, 1.0f);
                corte = 1;
            }
        }
    }
    if (limitado != 0) {
        *limitado = corte;
    }
    return t;
}

static float vel_roda_eixo(const Attitude3 *vel_rodas, int eixo) {
    if (vel_rodas == 0) {
        return NAN;
    }
    if (eixo == 0) {
        return vel_rodas->roll;
    }
    if (eixo == 1) {
        return vel_rodas->pitch;
    }
    return vel_rodas->yaw;
}

int calcular_comandos_rodas(const AttitudeEstimate *estado,
                            const Attitude3 *atitude_alvo,
                            const Attitude3 *vel_rodas,
                            float dt,
                            WheelCommands *comandos) {
    if (comandos == 0) {
        return 0;
    }
    comandos->roll.velocidade_angular_corpo_desejada = 0.0f;
    comandos->roll.torque_roda = 0.0f;
    comandos->roll.limitado = 0;
    comandos->pitch.velocidade_angular_corpo_desejada = 0.0f;
    comandos->pitch.torque_roda = 0.0f;
    comandos->pitch.limitado = 0;
    comandos->yaw.velocidade_angular_corpo_desejada = 0.0f;
    comandos->yaw.torque_roda = 0.0f;
    comandos->yaw.limitado = 0;

    if (estado == 0 || atitude_alvo == 0 ||
        !isfinite(dt) || dt < DT_MIN_S ||
        !isfinite(estado->atitude.yaw) ||
        !isfinite(estado->velocidade_angular.roll) ||
        !isfinite(estado->velocidade_angular.pitch) ||
        !isfinite(estado->velocidade_angular.yaw) ||
        !isfinite(atitude_alvo->yaw)) {
        return 0;
    }

    /* Roll e pitch: apenas amortecimento (nao ha setpoint de atitude). */
    comandos->roll.torque_roda = proteger_saturacao_roda(
        calcular_amortecimento_roll(estado->velocidade_angular.roll),
        vel_roda_eixo(vel_rodas, 0), &comandos->roll.limitado);
    comandos->pitch.torque_roda = proteger_saturacao_roda(
        calcular_amortecimento_pitch(estado->velocidade_angular.pitch),
        vel_roda_eixo(vel_rodas, 1), &comandos->pitch.limitado);

    /* Yaw: cascata posicao -> velocidade -> binario. */
    float yaw_medido = envolver_angulo(estado->atitude.yaw);
    float erro_yaw = envolver_angulo(envolver_angulo(atitude_alvo->yaw) -
                                     yaw_medido);

    int congelar_externa = pid_velocidade_yaw.saturado ||
                           roda_yaw_limitada_anterior;
    comandos->yaw.velocidade_angular_corpo_desejada =
        calcular_malha_externa_yaw(erro_yaw, yaw_medido, dt,
                                   congelar_externa);

    float torque = calcular_malha_intermedia_yaw(
        comandos->yaw.velocidade_angular_corpo_desejada,
        estado->velocidade_angular.yaw, dt, roda_yaw_limitada_anterior);

    int roda_limitada = 0;
    comandos->yaw.torque_roda = proteger_saturacao_roda(
        torque, vel_roda_eixo(vel_rodas, 2), &roda_limitada);
    comandos->yaw.limitado = roda_limitada || pid_velocidade_yaw.saturado;
    roda_yaw_limitada_anterior = roda_limitada;
    return 1;
}

void imu_filter_init(IMU_Filter *filtro) {
    if (filtro != 0) {
        filtro->estimativa.atitude.roll = 0.0f;
        filtro->estimativa.atitude.pitch = 0.0f;
        filtro->estimativa.atitude.yaw = 0.0f;
        filtro->estimativa.velocidade_angular.roll = 0.0f;
        filtro->estimativa.velocidade_angular.pitch = 0.0f;
        filtro->estimativa.velocidade_angular.yaw = 0.0f;
        filtro->inicializado = 0;
    }
}

int imu_filter_update(IMU_Filter *filtro,
                      float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      float dt) {
    if (filtro == 0 || !isfinite(dt) || dt < DT_MIN_S || dt > IMU_DT_MAX_S ||
        !isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) {
        return 0;
    }

    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (!isfinite(accel_norm) || accel_norm < 1.0e-6f) {
        return 0;
    }
    /* Magnetometro opcional: ausente, a zeros ou invalido -> yaw so com giroscopio. */
    int usar_mag = isfinite(mx) && isfinite(my) && isfinite(mz);
    if (usar_mag) {
        float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
        usar_mag = isfinite(mag_norm) && mag_norm >= MAG_NORMA_MIN;
    }

    float roll_accel = atan2f(ay, az);
    float pitch_accel = atan2f(-ax, sqrtf(ay * ay + az * az));
    float roll_atual = filtro->estimativa.atitude.roll;
    float pitch_atual = filtro->estimativa.atitude.pitch;
    float yaw_atual = filtro->estimativa.atitude.yaw;
    float gx_filtrado = gx;
    float gy_filtrado = gy;
    float gz_filtrado = gz;
    /* Peso da correcao absoluta calculado a partir do tempo (independente da frequencia). */
    float peso_correcao = dt / (IMU_COMPLEMENTARY_TAU + dt);

    if (!filtro->inicializado) {
        roll_atual = roll_accel;
        pitch_atual = pitch_accel;
        yaw_atual = 0.0f;
    } else {
        float alpha_gyro = expf(-dt / IMU_GYRO_FILTER_TAU);
        gx_filtrado = alpha_gyro * filtro->estimativa.velocidade_angular.roll +
                      (1.0f - alpha_gyro) * gx;
        gy_filtrado = alpha_gyro * filtro->estimativa.velocidade_angular.pitch +
                      (1.0f - alpha_gyro) * gy;
        gz_filtrado = alpha_gyro * filtro->estimativa.velocidade_angular.yaw +
                      (1.0f - alpha_gyro) * gz;

        float delta_roll = gx_filtrado * dt;
        float delta_pitch = gy_filtrado * dt;
        float delta_yaw = gz_filtrado * dt;
        if (!isfinite(delta_roll) || !isfinite(delta_pitch) ||
            !isfinite(delta_yaw) || fabsf(delta_roll) > PI_F ||
            fabsf(delta_pitch) > PI_F || fabsf(delta_yaw) > PI_F) {
            return 0;
        }

        roll_atual = envolver_angulo(roll_atual + delta_roll);
        pitch_atual = envolver_angulo(pitch_atual + delta_pitch);
        yaw_atual = envolver_angulo(yaw_atual + delta_yaw);

        float accel_norm_g = accel_norm / IMU_GRAVITY;
        if (accel_norm_g >= ACCEL_MIN_G && accel_norm_g <= ACCEL_MAX_G) {
            roll_atual += peso_correcao *
                          envolver_angulo(roll_accel - roll_atual);
            pitch_atual += peso_correcao *
                           envolver_angulo(pitch_accel - pitch_atual);
        }
    }

    if (usar_mag) {
        float cos_roll = cosf(roll_atual);
        float sin_roll = sinf(roll_atual);
        float cos_pitch = cosf(pitch_atual);
        float sin_pitch = sinf(pitch_atual);
        /* Compensacao de inclinacao (ZYX). */
        float mx_nivelado = mx * cos_pitch +
                            (my * sin_roll + mz * cos_roll) * sin_pitch;
        float my_nivelado = my * cos_roll - mz * sin_roll;
        float yaw_magnetometro = atan2f(-my_nivelado, mx_nivelado);
        if (filtro->inicializado) {
            yaw_atual += peso_correcao *
                         envolver_angulo(yaw_magnetometro - yaw_atual);
        } else {
            yaw_atual = yaw_magnetometro;
        }
    }

    if (!isfinite(roll_atual) || !isfinite(pitch_atual) ||
        !isfinite(yaw_atual) || !isfinite(gx_filtrado) ||
        !isfinite(gy_filtrado) || !isfinite(gz_filtrado)) {
        return 0;
    }

    filtro->estimativa.atitude.roll = envolver_angulo(roll_atual);
    filtro->estimativa.atitude.pitch = envolver_angulo(pitch_atual);
    filtro->estimativa.atitude.yaw = envolver_angulo(yaw_atual);
    filtro->estimativa.velocidade_angular.roll = gx_filtrado;
    filtro->estimativa.velocidade_angular.pitch = gy_filtrado;
    filtro->estimativa.velocidade_angular.yaw = gz_filtrado;
    filtro->inicializado = 1;
    return 1;
}
