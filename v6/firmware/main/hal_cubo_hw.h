#ifndef HAL_CUBO_HW_H
#define HAL_CUBO_HW_H

/*
 * Arranque e diagnostico da camada de hardware (hal_cubo.c).
 * Separado de hal_cubo.h porque esse ficheiro e o contrato de chamada do
 * controlo e nao deve depender da placa.
 */

#include <stdint.h>

/* Inicia I2C, MPU-6050, magnetometro (se ativo) e a ligacao ao moteus.
 * Chamar UMA VEZ no app_main(), antes de app_controlo_iniciar().
 * Devolve 1 se a IMU respondeu; 0 caso contrario (sem IMU nao ha controlo). */
int hal_cubo_iniciar(void);

/* 1 se o binario esta a ser realmente aplicado ao motor; 0 em modo de bancada
 * (o binario e calculado e reportado, mas nao sai para o moteus). */
int hal_cubo_motor_ativo(void);

/* Contadores para a telemetria / depuracao. */
uint32_t hal_cubo_erros_i2c(void);
uint32_t hal_cubo_erros_can(void);
/* Ultimo codigo de falha lido do moteus (0 = sem falha). */
int hal_cubo_falha_moteus(void);

#endif
