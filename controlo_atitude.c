#include "controlo_atitude.h"
#include <math.h>

// Instanciação dos controladores para o eixo Yaw
static PID_Controller pid_posicao_yaw;
static PID_Controller pid_velocidade_yaw;

void init_controladores(void) {
    // Malha Externa (Posição / Alvo Visual -> Velocidade Angular Desejada)
    pid_posicao_yaw.kp = 2.0f; 
    pid_posicao_yaw.ki = 0.05f;
    pid_posicao_yaw.kd = 0.1f;
    pid_posicao_yaw.integral = 0.0f;
    pid_posicao_yaw.erro_anterior = 0.0f;
    pid_posicao_yaw.i_max = 5.0f; // Limite de saturação do termo integral de posição

    // Malha Interna (Velocidade Angular Real IMU -> Comando Torque para o Moteus)
    pid_velocidade_yaw.kp = 0.5f;
    pid_velocidade_yaw.ki = 0.1f;
    pid_velocidade_yaw.kd = 0.02f;
    pid_velocidade_yaw.integral = 0.0f;
    pid_velocidade_yaw.erro_anterior = 0.0f;
    pid_velocidade_yaw.i_max = MAX_TORQUE_COMMAND * 0.5f; // Anti-windup na malha interna
}

void reset_controladores(void) {
    pid_posicao_yaw.integral = 0.0f;
    pid_posicao_yaw.erro_anterior = 0.0f;
    
    pid_velocidade_yaw.integral = 0.0f;
    pid_velocidade_yaw.erro_anterior = 0.0f;
}

// 1. Malha Externa: Erro de Posição (pixels / graus) -> Vel. Angular Desejada (rad/s)
float calcular_malha_externa_yaw(float erro_posicao, float dt) {
    if (dt <= 0.0001f) return 0.0f; // Proteção contra divisão por zero

    // Acumulador Integral com Saturação (Clamping Anti-Windup)
    pid_posicao_yaw.integral += erro_posicao * dt;
    if (pid_posicao_yaw.integral > pid_posicao_yaw.i_max) {
        pid_posicao_yaw.integral = pid_posicao_yaw.i_max;
    } else if (pid_posicao_yaw.integral < -pid_posicao_yaw.i_max) {
        pid_posicao_yaw.integral = -pid_posicao_yaw.i_max;
    }

    // Derivada
    float derivada = (erro_posicao - pid_posicao_yaw.erro_anterior) / dt;
    pid_posicao_yaw.erro_anterior = erro_posicao;

    // Comando de velocidade desejada
    float vel_angular_desejada = (pid_posicao_yaw.kp * erro_posicao) + 
                                 (pid_posicao_yaw.ki * pid_posicao_yaw.integral) + 
                                 (pid_posicao_yaw.kd * derivada);

    // Saturação da saída da malha externa para segurança do satélite
    if (vel_angular_desejada > MAX_TARGET_VELOCITY) {
        vel_angular_desejada = MAX_TARGET_VELOCITY;
    } else if (vel_angular_desejada < -MAX_TARGET_VELOCITY) {
        vel_angular_desejada = -MAX_TARGET_VELOCITY;
    }

    return vel_angular_desejada;
}

// 2. Malha Interna: Vel. Angular -> Comando Torque Moteus r4.11 (N.m) com Anti-Windup Clamping
float calcular_malha_intermedia_yaw(float vel_angular_desejada, float vel_angular_real, float dt) {
    if (dt <= 0.0001f) return 0.0f; // Proteção contra divisão por zero

    float erro_vel = vel_angular_desejada - vel_angular_real;

    // Termos P e D
    float termo_p = pid_velocidade_yaw.kp * erro_vel;
    float derivada = (erro_vel - pid_velocidade_yaw.erro_anterior) / dt;
    float termo_d = pid_velocidade_yaw.kd * derivada;

    // Cálculo condicional do termo I com Anti-Windup
    float integracao_provisoria = pid_velocidade_yaw.integral + (erro_vel * dt);
    float termo_i = pid_velocidade_yaw.ki * integracao_provisoria;

    // Comando bruto (Torque pretendido)
    float comando_torque = termo_p + termo_i + termo_d;

    // Saturação e Anti-Windup Activo
    if (comando_torque > MAX_TORQUE_COMMAND) {
        comando_torque = MAX_TORQUE_COMMAND;
    } else if (comando_torque < -MAX_TORQUE_COMMAND) {
        comando_torque = -MAX_TORQUE_COMMAND;
    } else {
        // Atualiza a integral apenas quando não estiver saturado
        pid_velocidade_yaw.integral = integracao_provisoria;
    }

    pid_velocidade_yaw.erro_anterior = erro_vel;

    // Retorna comando de Torque a enviar via CAN-FD ao Moteus r4.11
    return comando_torque;
}

// 3. Eixos Roll e Pitch: Amortecimento Ativo de Oscilações
float calcular_damping_roll_pitch(float vel_angular_real, float ganho_damping) {
    // Resposta proporcional oposta à velocidade angular para dissipar energia
    return -ganho_damping * vel_angular_real;
}