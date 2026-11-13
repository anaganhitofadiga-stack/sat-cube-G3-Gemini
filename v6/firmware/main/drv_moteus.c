/*
 * drv_moteus.c — protocolo de registos do moteus sobre CAN-FD.
 *
 * NOTAS SOBRE O PROTOCOLO (confirmado na documentacao oficial do mjbots,
 * paginas "Raw CAN-FD", "Register Protocol" e "CAN Protocol", Nov 2026):
 *
 *  Identificador: numero de 16 bits = (fonte << 8) | destino, com o bit 15
 *  a pedir resposta. Host 0 -> moteus 1 com resposta = 0x8001. Como e
 *  > 0x7FFF, a trama tem de ser extendida. A resposta vem em 0x0100.
 *
 *  Corpo da trama: sequencia de subtramas. Cada subtrama comeca por um byte
 *  de comando:  base | (tipo << 2) | numero_de_registos
 *      base: 0x00 escrever, 0x10 ler, 0x20 resposta, 0x50 nop
 *      tipo: 0 = int8, 1 = int16, 2 = int32, 3 = f32
 *      numero_de_registos: 1..3 (0 = segue um varuint com o comprimento)
 *  Depois vem o numero do primeiro registo (varuint) e os dados
 *  (little-endian, registos consecutivos).
 *
 *  Registos usados aqui:
 *      0x000 modo          0x020 posicao comandada
 *      0x001 posicao       0x021 velocidade comandada
 *      0x002 velocidade    0x022 binario em feedforward
 *      0x003 binario       0x023 escala kp
 *      0x00f falha         0x024 escala kd
 *                          0x025 binario maximo
 *                          0x027 timeout do watchdog
 *  Modos: 0 = parado (escrever 0 tambem limpa falhas), 10 = posicao.
 *
 *  Unidades do moteus: posicao em voltas, velocidade em Hz (voltas/s),
 *  binario em N.m no eixo de saida. Logo rad/s = Hz * 2*pi.
 *
 *  Posicao = NaN em modo posicao significa "usa a posicao atual", o que, com
 *  kp_scale = kd_scale = 0, deixa passar apenas o binario de feedforward.
 */

#include "drv_moteus.h"

#include <math.h>
#include <string.h>

#define DOIS_PI              6.28318530717958647692f

#define REG_MODO             0x000
#define REG_POSICAO          0x001
#define REG_FALHA            0x00f
#define REG_CMD_POSICAO      0x020
#define REG_CMD_KP_ESCALA    0x023
#define REG_CMD_WATCHDOG     0x027

#define MODO_PARADO_MOTEUS   0
#define MODO_POSICAO_MOTEUS  10

#define CMD_ESCREVER         0x00
#define CMD_LER              0x10
#define CMD_RESPOSTA         0x20
#define CMD_NOP              0x50

#define TIPO_INT8            0
#define TIPO_F32             3

static int falha_moteus;
static int transporte_pronto;

/* Tamanhos de dados validos numa trama CAN-FD. */
static int arredondar_tamanho_fd(int n) {
    static const int tamanhos[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
                                   12, 16, 20, 24, 32, 48, 64};
    for (unsigned i = 0; i < sizeof(tamanhos) / sizeof(tamanhos[0]); ++i) {
        if (tamanhos[i] >= n) {
            return tamanhos[i];
        }
    }
    return 64;
}

static void por_f32(uint8_t *dados, int *i, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    dados[(*i)++] = (uint8_t)(bits & 0xff);
    dados[(*i)++] = (uint8_t)((bits >> 8) & 0xff);
    dados[(*i)++] = (uint8_t)((bits >> 16) & 0xff);
    dados[(*i)++] = (uint8_t)((bits >> 24) & 0xff);
}

static float obter_f32(const uint8_t *dados) {
    uint32_t bits = (uint32_t)dados[0] |
                    ((uint32_t)dados[1] << 8) |
                    ((uint32_t)dados[2] << 16) |
                    ((uint32_t)dados[3] << 24);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

static int tamanho_tipo(int tipo) {
    switch (tipo) {
    case 0: return 1;
    case 1: return 2;
    case 2: return 4;
    default: return 4;
    }
}

/* Le um varuint do moteus (7 bits por byte, bit alto = continua). */
static int obter_varuint(const uint8_t *dados, int n, int *i, uint32_t *valor) {
    uint32_t v = 0;
    int deslocamento = 0;
    for (int k = 0; k < 5; ++k) {
        if (*i >= n) {
            return 0;
        }
        uint8_t b = dados[(*i)++];
        v |= (uint32_t)(b & 0x7f) << deslocamento;
        if ((b & 0x80) == 0) {
            *valor = v;
            return 1;
        }
        deslocamento += 7;
    }
    return 0;
}

/*
 * Percorre as subtramas de resposta e extrai os registos que interessam.
 * Generico de proposito: nao assume a ordem nem o alinhamento da resposta.
 */
static void interpretar_resposta(const uint8_t *dados, int n,
                                 float *posicao_voltas, float *vel_hz,
                                 float *binario_nm, int *falha) {
    int i = 0;
    while (i < n) {
        uint8_t comando = dados[i++];
        if (comando == CMD_NOP || comando == 0x00) {
            continue;   /* enchimento */
        }
        if ((comando & 0x30) != CMD_RESPOSTA) {
            return;     /* nao e resposta: nao sabemos andar para a frente */
        }
        int tipo = (comando >> 2) & 0x03;
        uint32_t quantos = (uint32_t)(comando & 0x03);
        if (quantos == 0 && !obter_varuint(dados, n, &i, &quantos)) {
            return;
        }
        uint32_t primeiro_registo;
        if (!obter_varuint(dados, n, &i, &primeiro_registo)) {
            return;
        }
        int largura = tamanho_tipo(tipo);
        for (uint32_t k = 0; k < quantos; ++k) {
            if (i + largura > n) {
                return;
            }
            uint32_t reg = primeiro_registo + k;
            if (tipo == TIPO_F32) {
                float v = obter_f32(&dados[i]);
                if (reg == 0x001 && posicao_voltas) *posicao_voltas = v;
                if (reg == 0x002 && vel_hz)         *vel_hz = v;
                if (reg == 0x003 && binario_nm)     *binario_nm = v;
            } else if (largura == 1) {
                if (reg == REG_FALHA && falha) {
                    *falha = (int)(int8_t)dados[i];
                }
            }
            i += largura;
        }
    }
}

/* Envia a trama e espera a resposta do moteus. Devolve 1 se leu a resposta. */
static int trocar(const uint8_t *pedido, int n_pedido, float *vel_roda_rad_s) {
    if (!transporte_pronto) {
        return 0;
    }
    uint32_t id = 0x8000u | ((uint32_t)(MOTEUS_ID_HOST & 0x7f) << 8) |
                  (uint32_t)(MOTEUS_ID & 0x7f);
    if (!mcp2518fd_enviar(id, 1, pedido, n_pedido)) {
        return 0;
    }

    /* A resposta vem com fonte e destino trocados e sem o bit de pedido. */
    uint32_t id_esperado = ((uint32_t)(MOTEUS_ID & 0x7f) << 8) |
                           (uint32_t)(MOTEUS_ID_HOST & 0x7f);
    for (int tentativa = 0; tentativa < 4; ++tentativa) {
        uint32_t id_recebido = 0;
        uint8_t resposta[64];
        int n = 0;
        if (!mcp2518fd_receber(&id_recebido, resposta, (int)sizeof(resposta),
                               &n, MOTEUS_TIMEOUT_US)) {
            return 0;
        }
        if (id_recebido != id_esperado) {
            continue;   /* trama de outro no */
        }
        float vel_hz = NAN;
        interpretar_resposta(resposta, n, NULL, &vel_hz, NULL, &falha_moteus);
        if (vel_roda_rad_s) {
            *vel_roda_rad_s = isfinite(vel_hz)
                                  ? (float)MOTEUS_SINAL * vel_hz * DOIS_PI
                                  : NAN;
        }
        return 1;
    }
    return 0;
}

/* Subtramas de leitura comuns: velocidade/posicao/binario + falha. */
static int acrescentar_leituras(uint8_t *d, int i) {
    d[i++] = CMD_LER | (TIPO_F32 << 2) | 3;   /* 0x1f */
    d[i++] = REG_POSICAO;                     /* 0x001..0x003 */
    d[i++] = CMD_LER | (TIPO_INT8 << 2) | 1;  /* 0x11 */
    d[i++] = REG_FALHA;
    return i;
}

int drv_moteus_iniciar(void) {
    falha_moteus = 0;
    transporte_pronto = mcp2518fd_iniciar() ? 1 : 0;
    if (!transporte_pronto) {
        return 0;
    }
    /* Escrever modo 0 limpa falhas pendentes de um arranque anterior. */
    float v = NAN;
    (void)drv_moteus_parar(&v);
    return drv_moteus_parar(&v);
}

int drv_moteus_comandar_binario(float binario_nm, float max_binario_nm,
                                float watchdog_s, float *vel_roda_rad_s) {
    if (vel_roda_rad_s) {
        *vel_roda_rad_s = NAN;
    }
    if (!isfinite(binario_nm)) {
        return drv_moteus_parar(vel_roda_rad_s);
    }

    uint8_t d[64];
    int i = 0;

    /* modo = posicao */
    d[i++] = CMD_ESCREVER | (TIPO_INT8 << 2) | 1;   /* 0x01 */
    d[i++] = REG_MODO;
    d[i++] = MODO_POSICAO_MOTEUS;

    /* posicao = NaN, velocidade = 0, binario em feedforward */
    d[i++] = CMD_ESCREVER | (TIPO_F32 << 2) | 3;    /* 0x0f */
    d[i++] = REG_CMD_POSICAO;
    por_f32(d, &i, NAN);
    por_f32(d, &i, 0.0f);
    por_f32(d, &i, (float)MOTEUS_SINAL * binario_nm);

    /* kp_scale = 0, kd_scale = 0, binario maximo */
    d[i++] = CMD_ESCREVER | (TIPO_F32 << 2) | 3;    /* 0x0f */
    d[i++] = REG_CMD_KP_ESCALA;
    por_f32(d, &i, 0.0f);
    por_f32(d, &i, 0.0f);
    por_f32(d, &i, isfinite(max_binario_nm) ? fabsf(max_binario_nm) : NAN);

    /* watchdog: se o ESP32 deixar de comandar, o moteus para sozinho */
    d[i++] = CMD_ESCREVER | (TIPO_F32 << 2) | 1;    /* 0x0d */
    d[i++] = REG_CMD_WATCHDOG;
    por_f32(d, &i, isfinite(watchdog_s) ? watchdog_s : 0.1f);

    i = acrescentar_leituras(d, i);

    int n = arredondar_tamanho_fd(i);
    for (int k = i; k < n; ++k) {
        d[k] = CMD_NOP;
    }
    return trocar(d, n, vel_roda_rad_s);
}

int drv_moteus_parar(float *vel_roda_rad_s) {
    if (vel_roda_rad_s) {
        *vel_roda_rad_s = NAN;
    }
    uint8_t d[16];
    int i = 0;
    d[i++] = CMD_ESCREVER | (TIPO_INT8 << 2) | 1;
    d[i++] = REG_MODO;
    d[i++] = MODO_PARADO_MOTEUS;
    i = acrescentar_leituras(d, i);
    int n = arredondar_tamanho_fd(i);
    for (int k = i; k < n; ++k) {
        d[k] = CMD_NOP;
    }
    return trocar(d, n, vel_roda_rad_s);
}

int drv_moteus_ultima_falha(void) {
    return falha_moteus;
}
