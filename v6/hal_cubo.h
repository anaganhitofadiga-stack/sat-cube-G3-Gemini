#ifndef HAL_CUBO_H
#define HAL_CUBO_H

/*
 * Contrato entre o controlo (Ana) e os drivers de hardware (Edson).
 * O controlo so fala com o hardware atraves destas funcoes. No simulador
 * (MSYS2) elas sao substituidas por versoes falsas; no ESP32 o Edson
 * implementa-as em hal_cubo.c com os drivers I2C / SPI / CAN-FD reais.
 *
 * Convencoes (OBRIGATORIAS — confirmar na montagem fisica):
 *  - Eixos do corpo: x = roll, y = pitch, z = yaw (z para cima).
 *  - Unidades SI: m/s^2, rad/s, N.m, s.
 *  - Binario positivo da roda acelera-a no sentido +z; o corpo recebe o
 *    binario simetrico (gira em -z).
 */

#include <stdint.h>

typedef struct {
    float ax, ay, az;   /* m/s^2, eixos do corpo */
    float gx, gy, gz;   /* rad/s, eixos do corpo, SEM correcao de vies (o app faz) */
} AmostraIMU;

/* Tempo monotono em microssegundos (ESP32: esp_timer_get_time()).
 * A camara TEM de usar o mesmo relogio para o t_captura_us. */
int64_t hal_tempo_us(void);

/* MPU-6050 por I2C. Devolve 1 se a leitura e valida. Duracao < 1 ms. */
int hal_imu_ler(AmostraIMU *amostra);

/* LIS3MDL (opcional). Devolve 0 se nao existir ou falhar. Valores calibrados
 * e ja rodados para os eixos do corpo. */
int hal_mag_ler(float *mx, float *my, float *mz);

/*
 * Moteus r4.11 por CAN-FD (via MCP2518FD/SPI).
 * Envia um comando de binario puro e devolve a velocidade da roda lida na
 * resposta (rad/s; o moteus devolve rev/s -> multiplicar por 2*pi).
 * Sugestao de implementacao: modo "position" com kp_scale = 0, kd_scale = 0,
 * position = NaN, velocity = 0, feedforward_torque = binario_nm,
 * maximum_torque = MAX_TORQUE_COMMAND e watchdog_timeout ~0,1 s
 * (confirmar nomes/registos na documentacao do moteus).
 * Devolve 1 se a trama foi enviada e a resposta recebida.
 */
int hal_moteus_comandar_binario(float binario_nm, float *vel_roda_rad_s);

/* Coloca o moteus em modo "stop" (sem corrente). Le a velocidade se puder. */
int hal_moteus_parar(float *vel_roda_rad_s);

#endif
