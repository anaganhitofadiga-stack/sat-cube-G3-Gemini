#include "controlo_visual.h"
#include "controlo_atitude.h"
#include <math.h>

#define GRAUS_PARA_RAD 0.01745329252f
#define PI_F 3.14159265358979323846f

int converter_erro_visual_rad(float erro, UnidadeVisual unidade,
                              float distancia_focal_px, int sinal_camera,
                              float *erro_rad) {
    if (erro_rad == 0 || !isfinite(erro) ||
        (sinal_camera != 1 && sinal_camera != -1)) {
        return 0;
    }
    float angulo;
    if (unidade == VISUAL_UNIDADE_GRAUS) {
        angulo = erro * GRAUS_PARA_RAD;
    } else if (unidade == VISUAL_UNIDADE_PIXEIS) {
        if (!isfinite(distancia_focal_px) || distancia_focal_px <= 0.0f) {
            return 0;
        }
        /* Modelo pinhole: angulo = atan(pixeis / distancia focal em pixeis). */
        angulo = atan2f(erro, distancia_focal_px);
    } else {
        return 0;
    }
    angulo *= (float)sinal_camera;
    if (!isfinite(angulo) || fabsf(angulo) >= 0.5f * PI_F) {
        return 0;
    }
    *erro_rad = angulo;
    return 1;
}

int atualizar_alvo_yaw_visual(float erro_pixeis,
                              float distancia_focal_px,
                              float yaw_captura_rad,
                              int sinal_camera,
                              float idade_imagem_s,
                              float *yaw_alvo_rad) {
    float erro_angular;
    if (yaw_alvo_rad == 0 || !isfinite(yaw_captura_rad) ||
        !isfinite(idade_imagem_s) || idade_imagem_s < 0.0f ||
        idade_imagem_s > VISUAL_IDADE_MAX_S ||
        !converter_erro_visual_rad(erro_pixeis, VISUAL_UNIDADE_PIXEIS,
                                   distancia_focal_px, sinal_camera,
                                   &erro_angular)) {
        return 0;
    }
    float alvo = envolver_angulo(yaw_captura_rad) + erro_angular;
    if (!isfinite(alvo)) {
        return 0;
    }
    *yaw_alvo_rad = envolver_angulo(alvo);
    return 1;
}

void historico_yaw_init(HistoricoYaw *h) {
    if (h != 0) {
        h->proximo = 0;
        h->contagem = 0;
    }
}

void historico_yaw_registar(HistoricoYaw *h, int64_t t_us, float yaw) {
    if (h == 0 || !isfinite(yaw)) {
        return;
    }
    if (h->contagem > 0) {
        int ultimo = (h->proximo - 1 + HISTORICO_YAW_TAMANHO) %
                     HISTORICO_YAW_TAMANHO;
        if (t_us <= h->t_us[ultimo]) {
            return; /* tempo nao crescente: ignora */
        }
    }
    h->t_us[h->proximo] = t_us;
    h->yaw[h->proximo] = envolver_angulo(yaw);
    h->proximo = (h->proximo + 1) % HISTORICO_YAW_TAMANHO;
    if (h->contagem < HISTORICO_YAW_TAMANHO) {
        h->contagem++;
    }
}

int historico_yaw_obter(const HistoricoYaw *h, int64_t t_us, float *yaw) {
    if (h == 0 || yaw == 0 || h->contagem == 0) {
        return 0;
    }
    const int n = HISTORICO_YAW_TAMANHO;
    int recente = (h->proximo - 1 + n) % n;
    int antigo = (h->proximo - h->contagem + n) % n;

    if (t_us >= h->t_us[recente]) {
        if (t_us - h->t_us[recente] <= HISTORICO_FUTURO_MAX_US) {
            *yaw = h->yaw[recente];
            return 1;
        }
        return 0;
    }
    if (t_us < h->t_us[antigo]) {
        return 0; /* mais antigo do que o historico guardado */
    }
    for (int k = 0; k < h->contagem - 1; ++k) {
        int i1 = (recente - k + n) % n;
        int i0 = (i1 - 1 + n) % n;
        if (t_us >= h->t_us[i0]) {
            int64_t intervalo = h->t_us[i1] - h->t_us[i0];
            if (intervalo <= 0) {
                *yaw = h->yaw[i1];
                return 1;
            }
            float f = (float)(t_us - h->t_us[i0]) / (float)intervalo;
            /* Interpolacao pelo caminho mais curto (funciona ao cruzar +-180 graus). */
            float d = envolver_angulo(h->yaw[i1] - h->yaw[i0]);
            *yaw = envolver_angulo(h->yaw[i0] + f * d);
            return 1;
        }
    }
    return 0;
}

int rastreador_init(RastreadorVisual *r, float distancia_focal_px,
                    int sinal_camera) {
    if (r == 0 || !isfinite(distancia_focal_px) ||
        distancia_focal_px <= 0.0f ||
        (sinal_camera != 1 && sinal_camera != -1)) {
        return 0;
    }
    r->distancia_focal_px = distancia_focal_px;
    r->sinal_camera = sinal_camera;
    r->offset_alvo_rad = 0.0f;
    r->imagens_aceites = 0;
    r->imagens_rejeitadas = 0;
    rastreador_reiniciar(r);
    return 1;
}

void rastreador_reiniciar(RastreadorVisual *r) {
    if (r == 0) {
        return;
    }
    r->estado = VISUAL_SEM_ALVO;
    r->yaw_alvo = 0.0f;
    r->alvo_definido = 0;
    r->t_ultima_valida_us = 0;
    r->confirmacoes = 0;
    r->yaw_candidato = 0.0f;
}

void rastreador_definir_offset(RastreadorVisual *r, float offset_rad) {
    if (r != 0 && isfinite(offset_rad) && fabsf(offset_rad) < 0.5f * PI_F) {
        r->offset_alvo_rad = offset_rad;
    }
}

int rastreador_processar_medida(RastreadorVisual *r, const HistoricoYaw *h,
                                const MedidaVisual *m, int64_t agora_us) {
    if (r == 0 || h == 0 || m == 0 || !m->alvo_detetado) {
        return 0;
    }
    float idade_s = (float)(agora_us - m->t_captura_us) * 1.0e-6f;
    float yaw_captura;
    float erro_rad;
    if (idade_s < -1.0e-6f * (float)HISTORICO_FUTURO_MAX_US ||
        idade_s > VISUAL_IDADE_MAX_S ||
        !historico_yaw_obter(h, m->t_captura_us, &yaw_captura) ||
        !converter_erro_visual_rad(m->erro_horizontal,
                                   (UnidadeVisual)m->unidade,
                                   r->distancia_focal_px, r->sinal_camera,
                                   &erro_rad)) {
        r->imagens_rejeitadas++;
        return 0;
    }

    /* Alvo absoluto = onde o cubo apontava na captura + desvio visto na imagem. */
    float novo = envolver_angulo(yaw_captura + erro_rad - r->offset_alvo_rad);

    /* Rejeicao de "falsos alvos": um salto grande durante o seguimento so e
     * aceite se se repetir em VISUAL_CONFIRMACOES imagens seguidas. */
    if (r->estado == VISUAL_A_SEGUIR && r->alvo_definido &&
        fabsf(envolver_angulo(novo - r->yaw_alvo)) > VISUAL_SALTO_MAX_RAD) {
        if (r->confirmacoes > 0 &&
            fabsf(envolver_angulo(novo - r->yaw_candidato)) <=
                VISUAL_TOLERANCIA_CONF_RAD) {
            r->confirmacoes++;
        } else {
            r->confirmacoes = 1;
        }
        r->yaw_candidato = novo;
        if (r->confirmacoes < VISUAL_CONFIRMACOES) {
            r->imagens_rejeitadas++;
            return 0;
        }
    }

    r->confirmacoes = 0;
    r->yaw_alvo = novo;
    r->alvo_definido = 1;
    r->estado = VISUAL_A_SEGUIR;
    r->t_ultima_valida_us = m->t_captura_us;
    r->imagens_aceites++;
    return 1;
}

EstadoVisual rastreador_obter_alvo(RastreadorVisual *r, int64_t agora_us,
                                   float yaw_atual, float *yaw_alvo) {
    if (r == 0) {
        return VISUAL_SEM_ALVO;
    }
    if (!r->alvo_definido) {
        /* Ainda sem alvo: segura a orientacao atual em vez de girar as cegas. */
        if (isfinite(yaw_atual)) {
            r->yaw_alvo = envolver_angulo(yaw_atual);
            r->alvo_definido = 1;
        }
        r->estado = VISUAL_SEM_ALVO;
    } else if (r->estado != VISUAL_SEM_ALVO) {
        float sem_imagem_s =
            (float)(agora_us - r->t_ultima_valida_us) * 1.0e-6f;
        if (sem_imagem_s > VISUAL_TIMEOUT_PERDIDO_S) {
            r->estado = VISUAL_SEM_ALVO;
        } else if (sem_imagem_s > VISUAL_TIMEOUT_OCLUSAO_S) {
            r->estado = VISUAL_OCLUSAO;
        }
    }
    if (yaw_alvo != 0 && r->alvo_definido) {
        *yaw_alvo = r->yaw_alvo;
    } else if (yaw_alvo != 0 && isfinite(yaw_atual)) {
        *yaw_alvo = envolver_angulo(yaw_atual);
    }
    return r->estado;
}
