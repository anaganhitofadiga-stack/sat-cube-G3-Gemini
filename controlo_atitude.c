#include "controlo_atitude.h"
#include <math.h>

/* Estados privados: o código que usa o módulo acede-lhes apenas pelas funções. */
static PID_Controller pid_posicao_yaw;
static PID_Controller pid_velocidade_yaw;

/* Ganhos de amortecimento carregados por init_controladores(). */
static float ganho_damping_roll;
static float ganho_damping_pitch;

/*
 * Restringe um valor a um intervalo fechado.
 * É usada para limitar integrais e comandos aos limites de segurança definidos.
 */
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
 * Valida uma amostra e o seu intervalo temporal antes dos cálculos do PID.
 * dt tem de ser finito e positivo para que integração e derivação façam sentido.
 */
static int entrada_valida(float valor, float dt) {
    return isfinite(valor) && isfinite(dt) && dt > 0.0f;
}

/*
 * Inicializa os ganhos e o estado dos controladores.
 * Deve ser chamada antes de calcular comandos; permite também voltar a carregar
 * os parâmetros definidos no cabeçalho depois de uma reconfiguração.
 */
void init_controladores(void) {
    /* Malha externa: converte erro de posição em velocidade angular desejada. */
    pid_posicao_yaw.kp = KP_POS_YAW;
    pid_posicao_yaw.ki = KI_POS_YAW;
    pid_posicao_yaw.kd = KD_POS_YAW;
    pid_posicao_yaw.integral = 0.0f;          // Começa sem erro acumulado.
    pid_posicao_yaw.erro_anterior = 0.0f;     // Referência inicial para a derivada.
    pid_posicao_yaw.i_max = IMAX_POS_YAW;     // Evita crescimento ilimitado da integral.

    /* Malha interna: converte erro de velocidade em binário para a roda. */
    pid_velocidade_yaw.kp = KP_VEL_YAW;
    pid_velocidade_yaw.ki = KI_VEL_YAW;
    pid_velocidade_yaw.kd = KD_VEL_YAW;
    pid_velocidade_yaw.integral = 0.0f;       // Apaga acumulação de execuções anteriores.
    pid_velocidade_yaw.erro_anterior = 0.0f;  // Mantido no estado, embora kd seja zero.
    pid_velocidade_yaw.i_max = IMAX_VEL_YAW;  // Limite do acumulador da malha interna.

    /* Os eixos passivos usam ganhos separados para permitir afinação independente. */
    ganho_damping_roll = GANHO_DAMPING_ROLL;
    ganho_damping_pitch = GANHO_DAMPING_PITCH;
}

/*
 * Reinicia apenas a memória dos controladores PID.
 * Mantém os ganhos e os ganhos de amortecimento; útil ao iniciar um novo ensaio.
 */
void reset_controladores(void) {
    pid_posicao_yaw.integral = 0.0f;       // Apaga o erro de posição acumulado.
    pid_posicao_yaw.erro_anterior = 0.0f;  // Evita derivada baseada no ensaio anterior.
    
    pid_velocidade_yaw.integral = 0.0f;       // Apaga o erro de velocidade acumulado.
    pid_velocidade_yaw.erro_anterior = 0.0f;  // Reinicia o histórico da malha interna.
}

/*
 * Malha externa de yaw: erro angular -> velocidade angular desejada.
 * A integral condicional impede windup quando a velocidade pedida já está
 * saturada e o erro continuaria a empurrar a saída para além do limite.
 */
float calcular_malha_externa_yaw(float erro_posicao, float dt) {
    /* Sem uma amostra válida não há cálculo seguro de integral ou derivada. */
    if (!entrada_valida(erro_posicao, dt)) {
        return 0.0f;
    }

    /* Candidato a integral, limitado antes de ser usado na saída PID. */
    float integral_provisoria = limitar(
        pid_posicao_yaw.integral + erro_posicao * dt,
        -pid_posicao_yaw.i_max,
        pid_posicao_yaw.i_max);

    /* Taxa de variação do erro angular, em rad/s. */
    float derivada = (erro_posicao - pid_posicao_yaw.erro_anterior) / dt;

    /* Soma proporcional, integral e derivativa: saída ainda não limitada (rad/s). */
    float saida_bruta = pid_posicao_yaw.kp * erro_posicao +
                        pid_posicao_yaw.ki * integral_provisoria +
                        pid_posicao_yaw.kd * derivada;

    /* Limita a velocidade que a malha interna pode tentar atingir. */
    float vel_angular_desejada = limitar(
        saida_bruta, -MAX_TARGET_VELOCITY, MAX_TARGET_VELOCITY);

    /*
     * Atualiza a integral se a saída não saturou, ou se o erro ajuda a sair
     * da saturação. Caso contrário, conserva o acumulador anterior (anti-windup).
     */
    if ((saida_bruta <= MAX_TARGET_VELOCITY &&
         saida_bruta >= -MAX_TARGET_VELOCITY) ||
        (saida_bruta > MAX_TARGET_VELOCITY && erro_posicao < 0.0f) ||
        (saida_bruta < -MAX_TARGET_VELOCITY && erro_posicao > 0.0f)) {
        pid_posicao_yaw.integral = integral_provisoria;
    }

    pid_posicao_yaw.erro_anterior = erro_posicao;  // Histórico para a próxima derivada.
    return vel_angular_desejada;
}

/*
 * Malha interna de yaw: erro de velocidade -> binário de comando da roda.
 * Com KD_VEL_YAW igual a zero, esta malha é PI. O sinal negativo converte
 * o binário desejado no corpo no binário aplicado à roda de reação.
 */
float calcular_malha_intermedia_yaw(float vel_angular_desejada, float vel_angular_real, float dt) {
    /* Valida as duas velocidades e o dt antes de alterar o estado do PI. */
    if (!entrada_valida(vel_angular_desejada, dt) ||
        !isfinite(vel_angular_real)) {
        return 0.0f;
    }

    /* Diferença entre velocidade pedida pela malha externa e medida pela IMU. */
    float erro_vel = vel_angular_desejada - vel_angular_real;

    /* Candidato à integral; limita o acumulador para evitar windup. */
    float integral_provisoria = limitar(
        pid_velocidade_yaw.integral + erro_vel * dt,
        -pid_velocidade_yaw.i_max,
        pid_velocidade_yaw.i_max);

    /* Binário equivalente pedido ao corpo antes de aplicar o limite do atuador. */
    float binario_corpo = pid_velocidade_yaw.kp * erro_vel +
                          pid_velocidade_yaw.ki * integral_provisoria;

    /* A roda recebe o binário oposto; satura no limite máximo permitido. */
    float binario_roda = limitar(
        -binario_corpo, -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND);

    /* Anti-windup: só integra se não saturou ou se o erro reduz a saturação. */
    if ((binario_corpo <= MAX_TORQUE_COMMAND &&
         binario_corpo >= -MAX_TORQUE_COMMAND) ||
        (binario_corpo > MAX_TORQUE_COMMAND && erro_vel < 0.0f) ||
        (binario_corpo < -MAX_TORQUE_COMMAND && erro_vel > 0.0f)) {
        pid_velocidade_yaw.integral = integral_provisoria;
    }

    return binario_roda;
}

/*
 * Amortece roll: produz binário de roda proporcional à velocidade medida.
 * O sinal da roda é escolhido para que o binário de reação no corpo se oponha
 * ao movimento; o comando também respeita o limite de torque.
 */
float calcular_amortecimento_roll(float vel_angular_real_roll) {
    /* Uma leitura inválida da IMU não deve produzir um comando de motor. */
    if (!isfinite(vel_angular_real_roll)) {
        return 0.0f;
    }

    /* Lei proporcional de amortecimento, limitada pelo atuador. */
    float comando_torque = ganho_damping_roll * vel_angular_real_roll;
    return limitar(comando_torque, -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND);
}

/*
 * Amortece pitch com a mesma lógica de roll, mantendo um ganho independente
 * para compensar diferenças mecânicas entre os eixos.
 */
float calcular_amortecimento_pitch(float vel_angular_real_pitch) {
    /* Evita transformar uma leitura não finita num comando inválido. */
    if (!isfinite(vel_angular_real_pitch)) {
        return 0.0f;
    }

    /* Binário de roda proporcional à velocidade e limitado por segurança. */
    float comando_torque = ganho_damping_pitch * vel_angular_real_pitch;
    return limitar(comando_torque, -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND);
}