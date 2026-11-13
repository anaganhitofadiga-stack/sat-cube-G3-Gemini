/*
 * main.c — arranque do firmware de bancada do cubo, V5.
 *
 * Ordem importante:
 *   1. hal_cubo_iniciar()      I2C + IMU + (moteus)
 *   2. app_usb_iniciar()       consola, para se verem as mensagens seguintes
 *   3. app_controlo_iniciar()  tarefa de 200 Hz; calibra o giroscopio primeiro,
 *                              por isso o cubo tem de estar PARADO ~2 s aqui.
 *
 * A camara ainda nao entra: este firmware testa a estimativa de atitude e a
 * cascata de controlo. Quando a OV5640 estiver pronta, acrescentar a tarefa
 * dela no Core 0 e chamar app_controlo_publicar_medida_visual() (exemplo no
 * fim de app_controlo.c).
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_controlo.h"
#include "app_usb.h"
#include "hal_cubo_hw.h"

static const char *TAG = "main";

void app_main(void) {
    ESP_LOGI(TAG, "Cubo V5 — firmware de bancada (controlo de atitude, yaw)");

    if (!hal_cubo_iniciar()) {
        ESP_LOGE(TAG, "hardware nao arrancou: sem IMU nao ha controlo");
        /* Continua para a consola ficar disponivel e podermos diagnosticar. */
    }
    if (!app_usb_iniciar()) {
        ESP_LOGE(TAG, "consola USB nao arrancou");
    }

    ESP_LOGW(TAG, "manter o cubo PARADO durante a calibracao do giroscopio");
    if (!app_controlo_iniciar()) {
        ESP_LOGE(TAG, "app_controlo_iniciar() falhou");
        return;
    }
    ESP_LOGI(TAG, "controlo a correr. '?' na consola para a lista de comandos");
}
