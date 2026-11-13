#ifndef DRV_MOTEUS_H
#define DRV_MOTEUS_H

/*
 * Camada de protocolo do moteus r4.11 sobre CAN-FD.
 *
 * Esta camada constroi e interpreta as tramas (protocolo de registos do
 * moteus, verificado na documentacao oficial: ver NOTAS no .c). Nao toca no
 * hardware: o transporte e feito pelas tres funcoes mcp2518fd_* declaradas no
 * fim deste ficheiro, que ficam por implementar (drv_mcp2518fd.c, Edson).
 *
 * Unidades desta interface: N.m e rad/s (a conversao de/para as unidades do
 * moteus — voltas e Hz — e feita aqui dentro).
 */

#include <stdint.h>

/* ID do moteus no barramento (valor de fabrica = 1). */
#ifndef MOTEUS_ID
#define MOTEUS_ID            1
#endif
/* ID deste no (host). Qualquer valor de 7 bits diferente do do moteus. */
#ifndef MOTEUS_ID_HOST
#define MOTEUS_ID_HOST       0
#endif
/*
 * Sinal do atuador. Trocar para -1 se, na montagem fisica, um binario
 * positivo comandado nao acelerar a roda no sentido +z do corpo.
 * Afeta o binario enviado E a velocidade lida, para ficarem coerentes.
 */
#ifndef MOTEUS_SINAL
#define MOTEUS_SINAL         (+1)
#endif
/* Tempo maximo de espera pela resposta do moteus. */
#ifndef MOTEUS_TIMEOUT_US
#define MOTEUS_TIMEOUT_US    1500
#endif

/* Abre o transporte e poe o moteus em "stop" (limpa falhas pendentes).
 * Devolve 1 em caso de sucesso. */
int drv_moteus_iniciar(void);

/*
 * Comando de binario puro em modo "position" com kp_scale = kd_scale = 0,
 * position = NaN e velocity = 0 — como sugerido em hal_cubo.h.
 *   binario_nm      binario pedido a roda (N.m)
 *   max_binario_nm  limite interno do moteus (segunda barreira)
 *   watchdog_s      se nao houver novo comando dentro deste tempo, o moteus
 *                   para sozinho. 0.1 e um bom valor.
 *   vel_roda_rad_s  saida: velocidade da roda lida na resposta
 * Devolve 1 se a trama foi enviada e a resposta recebida.
 */
int drv_moteus_comandar_binario(float binario_nm, float max_binario_nm,
                                float watchdog_s, float *vel_roda_rad_s);

/* Modo "stopped": sem corrente. Devolve 1 se houve resposta. */
int drv_moteus_parar(float *vel_roda_rad_s);

/* Ultimo codigo de falha reportado pelo moteus (registo 0x00f). */
int drv_moteus_ultima_falha(void);

/* ------------------------------------------------------------------------
 * TRANSPORTE — a implementar em drv_mcp2518fd.c (Edson).
 *
 * Requisitos do moteus (documentacao oficial):
 *  - CAN-FD, bit rate 1 Mbit/s na arbitragem e 5 Mbit/s nos dados,
 *    ponto de amostragem 0,666, SJW/DSJW tao grandes quanto possivel.
 *  - Trama de dados (nao remota), BRS ligado, sem ESI.
 *  - Identificador: usar trama EXTENDIDA sempre que o ID > 0x7FFF
 *    (e o nosso caso, porque o bit 15 pede resposta).
 *  - Terminacao de 120 ohm no barramento.
 * ------------------------------------------------------------------------ */

/* Configura o MCP2518FD (SPI nos pinos de placa_xiao.h) e o bit timing acima.
 * Devolve 1 em caso de sucesso. */
int mcp2518fd_iniciar(void);

/* Envia uma trama CAN-FD. 'n' e o numero de bytes uteis (ja vem num tamanho
 * valido de CAN-FD: 0..8, 12, 16, 20, 24, 32, 48, 64).
 * Devolve 1 se a trama foi entregue ao controlador. */
int mcp2518fd_enviar(uint32_t id, int id_extendido, const uint8_t *dados, int n);

/* Recebe a proxima trama, esperando no maximo timeout_us.
 * Devolve 1 se recebeu, 0 se expirou o tempo. */
int mcp2518fd_receber(uint32_t *id, uint8_t *dados, int max_bytes, int *n,
                      int timeout_us);

#endif
