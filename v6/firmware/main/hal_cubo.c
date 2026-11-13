/*
 * hal_cubo.c — implementacao do contrato hal_cubo.h no XIAO ESP32-S3 Sense.
 *
 *   relogio        esp_timer (o mesmo que a camara tem de usar no t_captura_us)
 *   IMU            MPU-6050 por I2C (GPIO5 SDA / GPIO6 SCL)
 *   magnetometro   LIS3MDL por I2C, OPCIONAL e desligado por omissao
 *   moteus         r4.11 por CAN-FD, atraves de drv_moteus.c
 *
 * MODO DE BANCADA (HAL_MOTEUS_MODO = 0, o que vem por omissao):
 *   o binario e calculado e aparece na telemetria, mas NAO e enviado ao
 *   motor. Serve para validar IMU, filtro, estimativa de yaw, cascata PID,
 *   telecomandos e telemetria so com o cabo USB, sem risco. Passar a 1 (e ter
 *   o drv_mcp2518fd.c pronto) para fechar a malha no motor.
 */

#include "hal_cubo.h"
#include "hal_cubo_hw.h"
#include "placa_xiao.h"
#include "drv_moteus.h"
#include "controlo_atitude.h"   /* MAX_TORQUE_COMMAND */

#include <math.h>
#include <string.h>

#include "esp_idf_version.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* A API de I2C mudou no ESP-IDF 5.2. Suportamos as duas. */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define HAL_I2C_NOVO 1
#include "driver/i2c_master.h"
#else
#define HAL_I2C_NOVO 0
#include "driver/i2c.h"
#endif

static const char *TAG = "hal";

/* ===================== Configuracao ===================== */

#define HAL_MOTEUS_MODO_BANCADA 0
#define HAL_MOTEUS_MODO_REAL    1
#ifndef HAL_MOTEUS_MODO
#define HAL_MOTEUS_MODO         HAL_MOTEUS_MODO_BANCADA
#endif

/*
 * Em bancada, 0 reporta a roda sempre parada (o mais simples para o primeiro
 * arranque) e 1 simula uma roda com inercia, o que exercita a protecao de
 * saturacao. Com 1, o cubo parado leva a roda simulada ao limite em poucos
 * segundos e dispara FALHA_SOBREVELOCIDADE — e o comportamento correto, mas
 * confunde quem esta so a verificar a IMU.
 */
#ifndef HAL_RODA_SIMULADA
#define HAL_RODA_SIMULADA       0
#endif
#define HAL_RODA_INERCIA        3.0e-4f   /* kg.m^2, estimativa */
#define HAL_RODA_ATRITO         1.0e-5f   /* N.m/(rad/s) */

/* Magnetometro: 1 so depois de calibrar o LIS3MDL longe do motor.
 * Manter coerente com USAR_MAGNETOMETRO em app_controlo.c. */
#ifndef HAL_MAG_ATIVO
#define HAL_MAG_ATIVO           0
#endif

/* Watchdog do moteus: sem comando novo dentro deste tempo, o motor para.
 * E a barreira que protege o cubo se o ESP32 bloquear ou reiniciar. */
#define HAL_MOTEUS_WATCHDOG_S   0.1f

/*
 * Remapeamento de eixos: para cada eixo do corpo (x = roll, y = pitch,
 * z = yaw, z para cima) indicar qual o eixo do sensor (0 = X, 1 = Y, 2 = Z)
 * e o sinal. Valores por omissao: sensor alinhado com o corpo.
 * CONFIRMAR NA MONTAGEM — e o passo 0 dos ensaios.
 */
#ifndef HAL_IMU_X_FONTE
#define HAL_IMU_X_FONTE   0
#endif
#ifndef HAL_IMU_X_SINAL
#define HAL_IMU_X_SINAL   (+1)
#endif
#ifndef HAL_IMU_Y_FONTE
#define HAL_IMU_Y_FONTE   1
#endif
#ifndef HAL_IMU_Y_SINAL
#define HAL_IMU_Y_SINAL   (+1)
#endif
#ifndef HAL_IMU_Z_FONTE
#define HAL_IMU_Z_FONTE   2
#endif
#ifndef HAL_IMU_Z_SINAL
#define HAL_IMU_Z_SINAL   (+1)
#endif

/* ===================== MPU-6050 ===================== */

#define MPU_ENDERECO        0x68   /* 0x69 se AD0 estiver a 1 */
#define MPU_WHO_AM_I        0x75
#define MPU_PWR_MGMT_1      0x6b
#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1a
#define MPU_GYRO_CONFIG     0x1b
#define MPU_ACCEL_CONFIG    0x1c
#define MPU_ACCEL_XOUT_H    0x3b

/* FS_SEL = 1: +-500 graus/s, 65,5 LSB por grau/s.
 * O corpo nunca passa de ~57 graus/s (MAX_TARGET_VELOCITY), mas a rotacao a
 * mao nos ensaios pode chegar perto deste limite. */
#define MPU_GYRO_FS_SEL     1
#define MPU_GYRO_LSB_POR_DPS 65.5f
/* AFS_SEL = 0: +-2 g, 16384 LSB por g. O cubo esta quase estatico. */
#define MPU_ACCEL_FS_SEL    0
#define MPU_ACCEL_LSB_POR_G 16384.0f

#define GRAUS_PARA_RAD      0.01745329252f

/* ===================== LIS3MDL ===================== */

#define LIS3MDL_ENDERECO_A  0x1c
#define LIS3MDL_ENDERECO_B  0x1e
#define LIS3MDL_WHO_AM_I    0x0f
#define LIS3MDL_ID          0x3d
#define LIS3MDL_CTRL_REG1   0x20
#define LIS3MDL_OUT_X_L     0x28
#define LIS3MDL_LSB_POR_GAUSS 6842.0f   /* escala +-4 gauss */

/* Viés de ferro duro, em contagens. Medir com o motor ligado e no lugar. */
#define LIS3MDL_OFFSET_X    0
#define LIS3MDL_OFFSET_Y    0
#define LIS3MDL_OFFSET_Z    0

/* ===================== Estado ===================== */

#if HAL_I2C_NOVO
typedef i2c_master_dev_handle_t hal_i2c_dev;
static i2c_master_bus_handle_t barramento;
#define HAL_I2C_DEV_INVALIDO NULL
#else
typedef int hal_i2c_dev;        /* endereco, ou -1 */
#define HAL_I2C_DEV_INVALIDO (-1)
#endif

static hal_i2c_dev dev_imu = HAL_I2C_DEV_INVALIDO;
static hal_i2c_dev dev_mag = HAL_I2C_DEV_INVALIDO;

static uint32_t contador_erros_i2c;
static uint32_t contador_erros_can;
static int imu_pronta;
static int mag_pronto;
static int motor_ativo;

/* Roda simulada em modo de bancada. */
static float roda_sim_vel;
static int64_t roda_sim_t_us;

/* ===================== Auxiliares de I2C ===================== */

static int i2c_escrever(hal_i2c_dev dev, uint8_t reg, uint8_t valor) {
    uint8_t buffer[2] = {reg, valor};
#if HAL_I2C_NOVO
    if (dev == NULL) {
        return 0;
    }
    if (i2c_master_transmit(dev, buffer, sizeof(buffer), 50) != ESP_OK) {
        contador_erros_i2c++;
        return 0;
    }
#else
    if (dev < 0) {
        return 0;
    }
    if (i2c_master_write_to_device(I2C_NUM_0, (uint8_t)dev, buffer,
                                   sizeof(buffer),
                                   pdMS_TO_TICKS(50)) != ESP_OK) {
        contador_erros_i2c++;
        return 0;
    }
#endif
    return 1;
}

static int i2c_ler(hal_i2c_dev dev, uint8_t reg, uint8_t *dados, size_t n) {
#if HAL_I2C_NOVO
    if (dev == NULL) {
        return 0;
    }
    if (i2c_master_transmit_receive(dev, &reg, 1, dados, n, 50) != ESP_OK) {
        contador_erros_i2c++;
        return 0;
    }
#else
    if (dev < 0) {
        return 0;
    }
    if (i2c_master_write_read_device(I2C_NUM_0, (uint8_t)dev, &reg, 1,
                                     dados, n,
                                     pdMS_TO_TICKS(50)) != ESP_OK) {
        contador_erros_i2c++;
        return 0;
    }
#endif
    return 1;
}

static int i2c_abrir_barramento(void) {
#if HAL_I2C_NOVO
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };
    return i2c_new_master_bus(&cfg, &barramento) == ESP_OK;
#else
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    if (i2c_param_config(I2C_NUM_0, &cfg) != ESP_OK) {
        return 0;
    }
    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK;
#endif
}

static int i2c_abrir_dispositivo(uint8_t endereco, hal_i2c_dev *dev) {
#if HAL_I2C_NOVO
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = endereco,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(barramento, &cfg, dev) == ESP_OK;
#else
    *dev = (int)endereco;
    return 1;
#endif
}

/* ===================== Relogio ===================== */

int64_t hal_tempo_us(void) {
    return esp_timer_get_time();
}

/* ===================== IMU ===================== */

static int mpu6050_iniciar(void) {
    if (!i2c_abrir_dispositivo(MPU_ENDERECO, &dev_imu)) {
        ESP_LOGE(TAG, "nao consegui abrir a MPU-6050 no I2C");
        return 0;
    }

    /* Reset e espera pelo arranque do sensor. */
    if (!i2c_escrever(dev_imu, MPU_PWR_MGMT_1, 0x80)) {
        ESP_LOGE(TAG, "MPU-6050 nao responde (verificar 3V3, GND, SDA/SCL)");
        return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    /* Relogio do PLL do giroscopio X: menos deriva do que o oscilador interno. */
    if (!i2c_escrever(dev_imu, MPU_PWR_MGMT_1, 0x01)) {
        return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t id = 0;
    if (!i2c_ler(dev_imu, MPU_WHO_AM_I, &id, 1)) {
        return 0;
    }
    /* 0x68 = MPU-6050 original; os clones devolvem 0x70, 0x72, 0x73, 0x98. */
    ESP_LOGI(TAG, "WHO_AM_I da IMU = 0x%02x", id);

    int ok = 1;
    /* DLPF a 42/44 Hz: corta o ruido mecanico sem atrasar muito a malha. */
    ok = i2c_escrever(dev_imu, MPU_CONFIG, 0x03) && ok;
    /* 1 kHz / (1 + 4) = 200 Hz, igual a frequencia da tarefa de controlo:
     * cada ciclo le uma amostra nova, sem repetidos nem saltos. */
    ok = i2c_escrever(dev_imu, MPU_SMPLRT_DIV, 0x04) && ok;
    ok = i2c_escrever(dev_imu, MPU_GYRO_CONFIG,
                      (uint8_t)(MPU_GYRO_FS_SEL << 3)) && ok;
    ok = i2c_escrever(dev_imu, MPU_ACCEL_CONFIG,
                      (uint8_t)(MPU_ACCEL_FS_SEL << 3)) && ok;
    if (!ok) {
        ESP_LOGE(TAG, "falha a configurar a MPU-6050");
        return 0;
    }
    ESP_LOGI(TAG, "MPU-6050 pronta: +-%d graus/s, +-2 g, 200 Hz",
             250 << MPU_GYRO_FS_SEL);
    return 1;
}

static int16_t juntar(const uint8_t *b) {
    return (int16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
}

static float escolher(const float v[3], int fonte, int sinal) {
    return (float)sinal * v[fonte];
}

int hal_imu_ler(AmostraIMU *amostra) {
    if (amostra == NULL || !imu_pronta) {
        return 0;
    }
    uint8_t b[14];
    if (!i2c_ler(dev_imu, MPU_ACCEL_XOUT_H, b, sizeof(b))) {
        return 0;
    }

    /* 0x3b..0x48: acelerometro (6), temperatura (2), giroscopio (6). */
    const float escala_a = IMU_GRAVITY / MPU_ACCEL_LSB_POR_G;
    const float escala_g = GRAUS_PARA_RAD / MPU_GYRO_LSB_POR_DPS;
    float a[3], g[3];
    for (int k = 0; k < 3; ++k) {
        a[k] = (float)juntar(&b[k * 2]) * escala_a;
        g[k] = (float)juntar(&b[8 + k * 2]) * escala_g;
    }

    /* Leitura toda a zeros: barramento em curto ou sensor desligado. */
    if (a[0] == 0.0f && a[1] == 0.0f && a[2] == 0.0f &&
        g[0] == 0.0f && g[1] == 0.0f && g[2] == 0.0f) {
        contador_erros_i2c++;
        return 0;
    }

    amostra->ax = escolher(a, HAL_IMU_X_FONTE, HAL_IMU_X_SINAL);
    amostra->ay = escolher(a, HAL_IMU_Y_FONTE, HAL_IMU_Y_SINAL);
    amostra->az = escolher(a, HAL_IMU_Z_FONTE, HAL_IMU_Z_SINAL);
    amostra->gx = escolher(g, HAL_IMU_X_FONTE, HAL_IMU_X_SINAL);
    amostra->gy = escolher(g, HAL_IMU_Y_FONTE, HAL_IMU_Y_SINAL);
    amostra->gz = escolher(g, HAL_IMU_Z_FONTE, HAL_IMU_Z_SINAL);
    return 1;
}

/* ===================== Magnetometro (opcional) ===================== */

static int lis3mdl_iniciar(void) {
    const uint8_t enderecos[2] = {LIS3MDL_ENDERECO_A, LIS3MDL_ENDERECO_B};
    for (int k = 0; k < 2; ++k) {
        hal_i2c_dev dev = HAL_I2C_DEV_INVALIDO;
        if (!i2c_abrir_dispositivo(enderecos[k], &dev)) {
            continue;
        }
        uint8_t id = 0;
        if (i2c_ler(dev, LIS3MDL_WHO_AM_I, &id, 1) && id == LIS3MDL_ID) {
            dev_mag = dev;
            int ok = 1;
            ok = i2c_escrever(dev, LIS3MDL_CTRL_REG1, 0x7c) && ok; /* UHP, 80 Hz */
            ok = i2c_escrever(dev, 0x21, 0x00) && ok;              /* +-4 gauss */
            ok = i2c_escrever(dev, 0x22, 0x00) && ok;              /* continuo */
            ok = i2c_escrever(dev, 0x23, 0x0c) && ok;              /* Z em UHP */
            ok = i2c_escrever(dev, 0x24, 0x40) && ok;              /* BDU */
            if (ok) {
                ESP_LOGI(TAG, "LIS3MDL em 0x%02x", enderecos[k]);
                return 1;
            }
        }
    }
    ESP_LOGW(TAG, "LIS3MDL nao encontrado: yaw so com giroscopio + camara");
    dev_mag = HAL_I2C_DEV_INVALIDO;
    return 0;
}

int hal_mag_ler(float *mx, float *my, float *mz) {
    if (mx == NULL || my == NULL || mz == NULL) {
        return 0;
    }
    *mx = 0.0f;
    *my = 0.0f;
    *mz = 0.0f;
    if (!mag_pronto) {
        return 0;   /* contrato: 0 = nao existe; o filtro usa so o giroscopio */
    }
    uint8_t b[6];
    /* Bit 7 do registo: auto-incremento no LIS3MDL. */
    if (!i2c_ler(dev_mag, LIS3MDL_OUT_X_L | 0x80, b, sizeof(b))) {
        return 0;
    }
    /* Little-endian, ao contrario da MPU-6050. */
    int16_t bruto[3];
    for (int k = 0; k < 3; ++k) {
        bruto[k] = (int16_t)((uint16_t)b[k * 2] | ((uint16_t)b[k * 2 + 1] << 8));
    }
    const int offsets[3] = {LIS3MDL_OFFSET_X, LIS3MDL_OFFSET_Y,
                            LIS3MDL_OFFSET_Z};
    float g[3];
    for (int k = 0; k < 3; ++k) {
        g[k] = (float)(bruto[k] - offsets[k]) / LIS3MDL_LSB_POR_GAUSS;
    }
    *mx = escolher(g, HAL_IMU_X_FONTE, HAL_IMU_X_SINAL);
    *my = escolher(g, HAL_IMU_Y_FONTE, HAL_IMU_Y_SINAL);
    *mz = escolher(g, HAL_IMU_Z_FONTE, HAL_IMU_Z_SINAL);
    return 1;
}

/* ===================== Moteus ===================== */

#if HAL_MOTEUS_MODO == HAL_MOTEUS_MODO_BANCADA
/* Roda simulada: dw/dt = binario / inercia - atrito * w / inercia. */
static float roda_simulada(float binario_nm) {
#if HAL_RODA_SIMULADA
    int64_t agora = esp_timer_get_time();
    float dt = (float)(agora - roda_sim_t_us) * 1.0e-6f;
    roda_sim_t_us = agora;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.1f) {
        return roda_sim_vel;
    }
    float t = isfinite(binario_nm) ? binario_nm : 0.0f;
    float aceleracao = (t - HAL_RODA_ATRITO * roda_sim_vel) / HAL_RODA_INERCIA;
    roda_sim_vel += aceleracao * dt;
    if (!isfinite(roda_sim_vel)) {
        roda_sim_vel = 0.0f;
    }
    return roda_sim_vel;
#else
    (void)binario_nm;
    roda_sim_t_us = esp_timer_get_time();
    roda_sim_vel = 0.0f;
    return 0.0f;
#endif
}
#endif  /* HAL_MOTEUS_MODO_BANCADA */

int hal_moteus_comandar_binario(float binario_nm, float *vel_roda_rad_s) {
    float vel = NAN;
    int ok;
#if HAL_MOTEUS_MODO == HAL_MOTEUS_MODO_REAL
    ok = drv_moteus_comandar_binario(binario_nm, MAX_TORQUE_COMMAND,
                                     HAL_MOTEUS_WATCHDOG_S, &vel);
    if (!ok) {
        contador_erros_can++;
    }
#else
    vel = roda_simulada(binario_nm);
    ok = 1;
#endif
    if (vel_roda_rad_s != NULL) {
        *vel_roda_rad_s = vel;
    }
    return ok;
}

int hal_moteus_parar(float *vel_roda_rad_s) {
    float vel = NAN;
    int ok;
#if HAL_MOTEUS_MODO == HAL_MOTEUS_MODO_REAL
    ok = drv_moteus_parar(&vel);
    if (!ok) {
        contador_erros_can++;
    }
#else
    vel = roda_simulada(0.0f);
    ok = 1;
#endif
    if (vel_roda_rad_s != NULL) {
        *vel_roda_rad_s = vel;
    }
    return ok;
}

/* ===================== Arranque ===================== */

int hal_cubo_iniciar(void) {
    contador_erros_i2c = 0;
    contador_erros_can = 0;
    imu_pronta = 0;
    mag_pronto = 0;
    motor_ativo = 0;
    roda_sim_vel = 0.0f;
    roda_sim_t_us = esp_timer_get_time();

    if (!i2c_abrir_barramento()) {
        ESP_LOGE(TAG, "nao consegui abrir o barramento I2C");
        return 0;
    }
    imu_pronta = mpu6050_iniciar();
    if (!imu_pronta) {
        return 0;
    }
#if HAL_MAG_ATIVO
    mag_pronto = lis3mdl_iniciar();
#else
    (void)lis3mdl_iniciar;   /* compilado mas nao usado */
    ESP_LOGI(TAG, "magnetometro desligado (HAL_MAG_ATIVO = 0)");
#endif

#if HAL_MOTEUS_MODO == HAL_MOTEUS_MODO_REAL
    motor_ativo = drv_moteus_iniciar();
    if (!motor_ativo) {
        ESP_LOGE(TAG, "moteus nao responde: verificar CAN-FD, terminacao e "
                      "alimentacao do motor");
    } else {
        ESP_LOGI(TAG, "moteus pronto (id %d)", MOTEUS_ID);
    }
#else
    ESP_LOGW(TAG, "MODO DE BANCADA: o binario e calculado mas NAO vai ao motor");
#endif
    return 1;
}

int hal_cubo_motor_ativo(void) {
    return motor_ativo;
}

uint32_t hal_cubo_erros_i2c(void) {
    return contador_erros_i2c;
}

uint32_t hal_cubo_erros_can(void) {
    return contador_erros_can;
}

int hal_cubo_falha_moteus(void) {
#if HAL_MOTEUS_MODO == HAL_MOTEUS_MODO_REAL
    return drv_moteus_ultima_falha();
#else
    return 0;
#endif
}
