#ifndef APP_USB_H
#define APP_USB_H

/*
 * Ponte entre o cabo USB e o controlo de atitude.
 *   - recebe telecomandos escritos a mao (ou pelo gs_cubo.py) e mete-os na
 *     fila do app_controlo;
 *   - imprime a telemetria em CSV, uma linha por amostra, prefixada por "T,".
 *
 * O XIAO ESP32-S3 nao tem chip USB-serie: usa-se USB Serial JTAG, por isso
 * sdkconfig.defaults tem de ter CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y.
 */

/* Cria a tarefa da consola. Chamar antes de app_controlo_iniciar() para ver
 * as mensagens da calibracao do giroscopio. Devolve 1 em caso de sucesso. */
int app_usb_iniciar(void);

#endif
