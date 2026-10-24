
#ifndef CONTROLO_ATITUDE_H
#define CONTROLO_ATITUDE_H

#include <stdint.h>
#include <stdbool.h>

// Limites do atuador (Moteus r4.11 com Motor mj5208, 330Kv)
// NOTA: O binário de pico do motor é 1.7 Nm. 
// O valor de 0.2 Nm é o limite de segurança inicial para evitar movimentos violentos no teste.
#define MAX_TORQUE_COMMAND   0.2f   // Torque máximo limitado em N.m

// Velocidade angular máxima desejada para o CubeSat (rad/s)
// 1.0 rad/s = ~57 graus/segundo (velocidade rápida, mas segura para testes terrestres)
#define MAX_TARGET_VELOCITY  1.0f

// Estrutura para armazenar o estado de cada controlador PID
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float erro_anterior;
    float i_max;            // Limite superior/inferior para o termo integral (Anti-Windup)
} PID_Controller;

// Protótipos das funções
void init_controladores(void);
void reset_controladores(void);

float calcular_malha_externa_yaw(float erro_posicao, float dt);
float calcular_malha_intermedia_yaw(float vel_angular_desejada, float vel_angular_real, float dt);
float calcular_damping_roll_pitch(float vel_angular_real, float ganho_damping);

#endif // CONTROLO_ATITUDE_H