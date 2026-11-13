#ifndef PLACA_XIAO_H
#define PLACA_XIAO_H

/*
 * Mapa de pinos do Seeed Studio XIAO ESP32-S3 Sense.
 * Segue a tabela de pinout acordada na equipa (email do Nuno Cruz).
 *
 * Atencao: a camara OV5640 do modulo Sense NAO usa estes pinos — liga pelo
 * conector board-to-board (GPIO 10..18, 38..40, 47, 48), por isso nao ha
 * conflito com os pads D0..D10.
 *
 * O XIAO nao tem chip USB-serie: o conector USB-C vai direto ao USB nativo
 * do ESP32-S3 (GPIO19/20). A consola e a telemetria usam USB Serial JTAG.
 */

/* Pad   GPIO   Funcao */
#define PIN_BATERIA_ADC   1    /* D0 — divisor 100k/13k, ADC1_CH0 */
#define PIN_BUZZER        2    /* D1 — saida digital, ativa a 1 */
#define PIN_CAN_CS        3    /* D2 — MCP2518FD CS (pull-up 10k externo) */
#define PIN_CAN_INT       4    /* D3 — MCP2518FD INT */
#define PIN_I2C_SDA       5    /* D4 — MPU-6050 + LIS3MDL */
#define PIN_I2C_SCL       6    /* D5 — MPU-6050 + LIS3MDL */
#define PIN_SPI_SCK       7    /* D8 */
#define PIN_SPI_MISO      8    /* D9 */
#define PIN_SPI_MOSI      9    /* D10 */

/* Velocidade do barramento I2C. Se aparecer FALHA_IMU com fios longos ou
 * breadboard, descer para 100000. */
#define I2C_FREQ_HZ       400000

#endif
