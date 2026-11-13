/*
 * teste_moteus.c — teste no PC do protocolo CAN-FD do moteus, sem hardware.
 *
 * Substitui o transporte (mcp2518fd_*) por versoes falsas, constroi os
 * comandos e verifica byte a byte que a trama esta conforme a documentacao
 * oficial do mjbots. Util para validar o drv_moteus.c antes de haver CAN, e
 * para nao o quebrar sem dar por isso.
 *
 * Compilar e correr no MSYS2 MINGW64 (ou em qualquer gcc):
 *   gcc -Wall -Wextra -I../firmware/main -o teste_moteus \
 *       teste_moteus.c ../firmware/main/drv_moteus.c -lm
 *   ./teste_moteus
 */

#include "drv_moteus.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static uint8_t ult_dados[64]; static int ult_n; static uint32_t ult_id, ult_ext;
static uint8_t resp[64]; static int resp_n; static uint32_t resp_id;
static int falhas_teste;

int mcp2518fd_iniciar(void) { return 1; }
int mcp2518fd_enviar(uint32_t id, int ext, const uint8_t *d, int n) {
    ult_id = id; ult_ext = (uint32_t)ext; ult_n = n; memcpy(ult_dados, d, (size_t)n); return 1;
}
int mcp2518fd_receber(uint32_t *id, uint8_t *d, int max, int *n, int to) {
    (void)max; (void)to; *id = resp_id; *n = resp_n; memcpy(d, resp, (size_t)resp_n); return 1;
}
static void por_f32(uint8_t *d, int *i, float v) {
    uint32_t b; memcpy(&b, &v, 4);
    d[(*i)++] = b & 0xff; d[(*i)++] = (b>>8)&0xff; d[(*i)++] = (b>>16)&0xff; d[(*i)++] = (b>>24)&0xff;
}
static float ler_f32(const uint8_t *d) { uint32_t b = d[0]|(d[1]<<8)|((uint32_t)d[2]<<16)|((uint32_t)d[3]<<24); float v; memcpy(&v,&b,4); return v; }
#define VERIFICA(cond, msg) do { if (!(cond)) { printf("  FALHOU: %s\n", msg); falhas_teste++; } } while (0)

int main(void) {
    /* resposta sintetica: 2f 01 <pos><vel><binario>, 21 0f 00 */
    int i = 0;
    resp[i++] = 0x2f; resp[i++] = 0x01;
    por_f32(resp, &i, 12.5f);      /* posicao, voltas */
    por_f32(resp, &i, 100.0f);     /* velocidade, Hz */
    por_f32(resp, &i, 0.07f);      /* binario */
    resp[i++] = 0x21; resp[i++] = 0x0f; resp[i++] = 0x00;  /* falha = 0 */
    resp_n = i; resp_id = 0x0100;

    printf("== arranque ==\n");
    VERIFICA(drv_moteus_iniciar() == 1, "iniciar devia dar 1");
    VERIFICA(ult_dados[2] == 0x00, "o arranque escreve modo=0 para limpar falhas");

    printf("== comando de binario ==\n");
    float vel = NAN;
    int ok = drv_moteus_comandar_binario(0.05f, 0.2f, 0.1f, &vel);
    VERIFICA(ok == 1, "devia ter tido resposta");
    printf("  id = 0x%04x (extendido=%u), %d bytes\n  ", (unsigned)ult_id, (unsigned)ult_ext, ult_n);
    for (int k = 0; k < ult_n; ++k) printf("%02x", ult_dados[k]);
    printf("\n");
    VERIFICA(ult_id == 0x8001, "id devia ser 0x8001 (host 0 -> moteus 1, com resposta)");
    VERIFICA(ult_ext == 1, "id > 0x7fff obriga a trama extendida");
    VERIFICA(ult_n == 48, "41 bytes uteis -> tamanho CAN-FD de 48");

    VERIFICA(ult_dados[0]==0x01 && ult_dados[1]==0x00 && ult_dados[2]==0x0a, "escrever modo=10 (posicao)");
    VERIFICA(ult_dados[3]==0x0f && ult_dados[4]==0x20, "escrever 3 f32 a partir de 0x020");
    VERIFICA(isnan(ler_f32(&ult_dados[5])), "posicao comandada = NaN");
    VERIFICA(ler_f32(&ult_dados[9]) == 0.0f, "velocidade comandada = 0");
    VERIFICA(ler_f32(&ult_dados[13]) == 0.05f, "binario em feedforward = 0,05");
    VERIFICA(ult_dados[17]==0x0f && ult_dados[18]==0x23, "escrever 3 f32 a partir de 0x023");
    VERIFICA(ler_f32(&ult_dados[19]) == 0.0f, "kp_scale = 0");
    VERIFICA(ler_f32(&ult_dados[23]) == 0.0f, "kd_scale = 0");
    VERIFICA(ler_f32(&ult_dados[27]) == 0.2f, "binario maximo = 0,2");
    VERIFICA(ult_dados[31]==0x0d && ult_dados[32]==0x27, "escrever 1 f32 em 0x027 (watchdog)");
    VERIFICA(ler_f32(&ult_dados[33]) == 0.1f, "watchdog = 0,1 s");
    VERIFICA(ult_dados[37]==0x1f && ult_dados[38]==0x01, "ler 3 f32 a partir de 0x001");
    VERIFICA(ult_dados[39]==0x11 && ult_dados[40]==0x0f, "ler int8 em 0x00f (falha)");
    for (int k = 41; k < 48; ++k) VERIFICA(ult_dados[k]==0x50, "enchimento com nop");

    printf("  velocidade lida = %.4f rad/s (esperado %.4f)\n", vel, 100.0f*6.283185307f);
    VERIFICA(fabsf(vel - 100.0f*6.283185307f) < 1e-2f, "Hz -> rad/s (x 2*pi)");
    VERIFICA(drv_moteus_ultima_falha() == 0, "falha do moteus = 0");

    printf("== comando de parar ==\n  ");
    ok = drv_moteus_parar(&vel);
    for (int k = 0; k < ult_n; ++k) printf("%02x", ult_dados[k]);
    printf("  (%d bytes)\n", ult_n);
    VERIFICA(ok == 1, "parar devia ter resposta");
    VERIFICA(ult_dados[0]==0x01 && ult_dados[1]==0x00 && ult_dados[2]==0x00, "escrever modo=0 (parado)");
    VERIFICA(ult_n == 7, "7 e um tamanho valido de CAN-FD: sem enchimento");

    printf("== falha reportada pelo moteus ==\n");
    resp[resp_n-1] = 33;   /* 33 = falha do driver do motor */
    (void)drv_moteus_parar(&vel);
    printf("  falha = %d\n", drv_moteus_ultima_falha());
    VERIFICA(drv_moteus_ultima_falha() == 33, "ler o registo de falha 0x00f");

    printf("== resposta de outro no e ignorada ==\n");
    resp_id = 0x0200;
    vel = 1234.0f;
    ok = drv_moteus_parar(&vel);
    VERIFICA(ok == 0, "id errado -> sem resposta valida");
    VERIFICA(isnan(vel), "velocidade fica NaN quando nao ha resposta");

    printf("\n%s (%d falha(s))\n", falhas_teste ? "COM ERROS" : "TODOS OS TESTES PASSARAM", falhas_teste);
    return falhas_teste != 0;
}
