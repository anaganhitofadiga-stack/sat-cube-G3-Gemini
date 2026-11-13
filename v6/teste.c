/*
 * teste.c — Ensaios SIL (Software-in-the-Loop) da V5.
 * Compilar:  gcc -Wall -Wextra -O2 -o teste teste.c controlo_atitude.c controlo_visual.c -lm
 *
 * Parte 1: testes unitarios (cada peca isolada).
 * Parte 2: simulacao do cubo com camara atrasada e os ensaios do enunciado.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "controlo_atitude.h"
#include "controlo_visual.h"

#define GRAUS(x) ((x) * 57.29577951f)
#define RAD(x)   ((x) * 0.01745329252f)

static int falhas_totais = 0;
#define VERIFICAR(cond, msg) do { \
    if (cond) { printf("  PASSOU: %s\n", msg); } \
    else { printf("  FALHOU: %s\n", msg); falhas_totais++; } } while (0)

/* ======================= PARTE 1: UNITARIOS ======================= */

static void teste_filtro_imu(void) {
    IMU_Filter f;
    imu_filter_init(&f);
    int ok = 1;
    for (int i = 0; i < 500; ++i) {
        ok &= imu_filter_update(&f, 0, 0, IMU_GRAVITY, 0, 0, 0.1f,
                                1.0f, 0, 0, 0.01f);
    }
    VERIFICAR(ok && fabsf(f.estimativa.atitude.yaw) < 0.08f &&
              fabsf(f.estimativa.atitude.roll) < 0.02f,
              "filtro complementar limita a deriva com magnetometro");

    imu_filter_init(&f);
    ok = 1;
    for (int i = 0; i < 200; ++i) {
        ok &= imu_filter_update(&f, 0, 0, IMU_GRAVITY, 0, 0, 0.1f,
                                0, 0, 0, 0.005f);
    }
    /* 1 s a 0,1 rad/s; o filtro passa-baixo atrasa ~0,05 s -> ~0,095 rad */
    VERIFICAR(ok && fabsf(f.estimativa.atitude.yaw - 0.095f) < 0.01f,
              "sem magnetometro o yaw integra o giroscopio (nao falha)");

    VERIFICAR(!imu_filter_update(&f, 0, 0, IMU_GRAVITY, 0, 0, 0, 0, 0, 0, 0.0f) &&
              !imu_filter_update(&f, 0, 0, IMU_GRAVITY, 0, 0, 0, 0, 0, 0, NAN) &&
              !imu_filter_update(&f, 0, 0, IMU_GRAVITY, 0, 0, 0, 0, 0, 0, 5.0f),
              "filtro rejeita dt = 0, NaN e dt enorme");
}

static void campo_no_corpo(float yaw, float pitch, float roll,
                           float *mx, float *my, float *mz) {
    float x = 0.4f, y = 0.0f, z = 0.9f, t;
    t = cosf(yaw) * x + sinf(yaw) * y;
    y = -sinf(yaw) * x + cosf(yaw) * y; x = t;
    t = cosf(pitch) * x - sinf(pitch) * z;
    z = sinf(pitch) * x + cosf(pitch) * z; x = t;
    t = cosf(roll) * y + sinf(roll) * z;
    z = -sinf(roll) * y + cosf(roll) * z; y = t;
    *mx = x; *my = y; *mz = z;
}

static void teste_compensacao_inclinacao(void) {
    const float yaw_real = 0.5236f, roll = 0.5236f, pitch = -0.4363f;
    float mx, my, mz;
    IMU_Filter f;
    campo_no_corpo(yaw_real, pitch, roll, &mx, &my, &mz);
    float ax = -IMU_GRAVITY * sinf(pitch);
    float ay = IMU_GRAVITY * cosf(pitch) * sinf(roll);
    float az = IMU_GRAVITY * cosf(pitch) * cosf(roll);
    imu_filter_init(&f);
    int ok = imu_filter_update(&f, ax, ay, az, 0, 0, 0, mx, my, mz, 0.01f);
    VERIFICAR(ok && fabsf(f.estimativa.atitude.yaw - yaw_real) < 0.01f,
              "rumo correto com roll 30 e pitch -25 graus");
}

static void teste_pid_dt(void) {
    PID_Controller p;
    inicializar_pid(&p, 1.0f, 1.0f, 0.1f, 10.0f);
    VERIFICAR(calcular_pid(&p, 1.0f, 0, 0.0f, -5, 5, 0) == 0.0f &&
              calcular_pid(&p, 1.0f, 0, 1e-9f, -5, 5, 0) == 0.0f &&
              calcular_pid(&p, 1.0f, 0, NAN, -5, 5, 0) == 0.0f &&
              calcular_pid(&p, 1.0f, 0, -0.01f, -5, 5, 0) == 0.0f,
              "PID devolve 0 com dt = 0, ~0, NaN ou negativo");
    reiniciar_pid(&p);
    calcular_pid(&p, 1.0f, 0, 10.0f, -5, 5, 0);
    VERIFICAR(fabsf(p.integral - DT_MAX_S) < 1e-6f,
              "dt de 10 s (tarefa bloqueada) e limitado a DT_MAX_S no integral");
}

static void teste_pid_derivada(void) {
    PID_Controller erro_d, medida_d;
    inicializar_pid(&erro_d, 1.0f, 0.0f, 0.1f, 1.0f);
    inicializar_pid(&medida_d, 1.0f, 0.0f, 0.1f, 1.0f);
    configurar_derivada_pid(&medida_d, 1, 1, 0.0f);
    calcular_pid(&erro_d, 0.0f, 0.0f, 0.005f, -100, 100, 0);
    calcular_pid(&medida_d, 0.0f, 0.0f, 0.005f, -100, 100, 0);
    /* O alvo salta 1 rad (nova imagem), o cubo ainda nao se mexeu. */
    float s_erro = calcular_pid(&erro_d, 1.0f, 0.0f, 0.005f, -100, 100, 0);
    float s_med = calcular_pid(&medida_d, 1.0f, 0.0f, 0.005f, -100, 100, 0);
    printf("  (salto de alvo: derivada no erro -> %.1f ; na medida -> %.1f)\n",
           s_erro, s_med);
    VERIFICAR(fabsf(s_med - 1.0f) < 1e-5f && s_erro > 20.0f,
              "derivada na medida elimina o pico quando o alvo visual salta");

    /* Medida angular a cruzar +-180 graus nao pode gerar derivada gigante. */
    reiniciar_pid(&medida_d);
    calcular_pid(&medida_d, 0.0f, 3.13f, 0.005f, -100, 100, 0);
    float s = calcular_pid(&medida_d, 0.0f, -3.13f, 0.005f, -100, 100, 0);
    VERIFICAR(fabsf(s) < 5.0f, "derivada correta ao cruzar +-180 graus");
}

static void teste_anti_windup(void) {
    PID_Controller p;
    inicializar_pid(&p, 0.0f, 1.0f, 0.0f, IMAX_VEL_YAW);
    for (int i = 0; i < 10; ++i) {
        calcular_pid(&p, 1.0f, 0, 0.05f, -MAX_TORQUE_COMMAND, MAX_TORQUE_COMMAND, 0);
    }
    VERIFICAR(p.integral <= IMAX_VEL_YAW + 1e-6f && p.saturado == 0,
              "integral limitado a IMAX e saida dentro do limite");

    inicializar_pid(&p, 1.0f, 1.0f, 0.0f, 100.0f);
    for (int i = 0; i < 100; ++i) {
        calcular_pid(&p, 5.0f, 0, 0.05f, -1.0f, 1.0f, 0);
    }
    float i_sat = p.integral;
    VERIFICAR(i_sat < 1e-3f && p.saturado,
              "integracao condicional: saturado, o integral nao cresce");

    inicializar_pid(&p, 0.1f, 1.0f, 0.0f, 100.0f);
    calcular_pid(&p, 1.0f, 0, 0.05f, -10.0f, 10.0f, 0);
    float antes = p.integral;
    calcular_pid(&p, 1.0f, 0, 0.05f, -10.0f, 10.0f, 1);
    float congelado = p.integral;
    calcular_pid(&p, -1.0f, 0, 0.05f, -10.0f, 10.0f, 1);
    VERIFICAR(congelado == antes && p.integral < congelado,
              "anti-windup em cascata: congela a subir, deixa descer");
}

static void teste_saturacao_roda(void) {
    int lim = 0;
    float t1 = proteger_saturacao_roda(1.0f, 0.0f, &lim);
    VERIFICAR(fabsf(t1 - MAX_TORQUE_COMMAND) < 1e-6f && lim,
              "binario limitado a MAX_TORQUE_COMMAND");
    float t2 = proteger_saturacao_roda(0.1f, MAX_WHEEL_SPEED_RAD_S, &lim);
    float t3 = proteger_saturacao_roda(-0.1f, MAX_WHEEL_SPEED_RAD_S, &lim);
    VERIFICAR(t2 == 0.0f && fabsf(t3 + 0.1f) < 1e-6f,
              "roda no limite: nao acelera mais, mas pode travar");
    float t4 = proteger_saturacao_roda(0.1f, 0.5f * (SOFT_WHEEL_SPEED_RAD_S +
                                       MAX_WHEEL_SPEED_RAD_S), &lim);
    VERIFICAR(fabsf(t4 - 0.05f) < 1e-3f, "reducao progressiva na zona de aviso");
    VERIFICAR(proteger_saturacao_roda(NAN, 0, &lim) == 0.0f && lim,
              "binario NaN vira 0");
}

static void teste_visual_unitario(void) {
    float y = 0.0f;
    VERIFICAR(atualizar_alvo_yaw_visual(50.0f, 500.0f, 0.2f, 1, 0.1f, &y) &&
              fabsf(y - (0.2f + atan2f(50.0f, 500.0f))) < 1e-5f,
              "conversao pixeis -> yaw (compatibilidade V4)");
    float e = 0.0f;
    VERIFICAR(converter_erro_visual_rad(10.0f, VISUAL_UNIDADE_GRAUS, 0, -1, &e) &&
              fabsf(e - RAD(-10.0f)) < 1e-6f,
              "conversao graus -> rad com sinal da camara");
    y = 123.0f;
    VERIFICAR(!atualizar_alvo_yaw_visual(50.0f, 500.0f, 0.2f, 1,
                                         VISUAL_IDADE_MAX_S + 0.01f, &y) &&
              y == 123.0f, "imagem demasiado antiga e rejeitada");

    HistoricoYaw h;
    historico_yaw_init(&h);
    historico_yaw_registar(&h, 1000, 3.10f);
    historico_yaw_registar(&h, 6000, -3.10f);  /* cruzou +180 */
    float yi = 0.0f;
    int ok = historico_yaw_obter(&h, 3500, &yi);
    VERIFICAR(ok && fabsf(fabsf(yi) - 3.14159f) < 0.01f,
              "historico interpola pelo caminho curto ao cruzar 180 graus");
    VERIFICAR(!historico_yaw_obter(&h, 500, &yi),
              "historico recusa instantes anteriores ao registo");

    /* Falso alvo: um salto de 40 graus isolado e ignorado. */
    RastreadorVisual r;
    rastreador_init(&r, 277.0f, 1);
    historico_yaw_init(&h);
    for (int i = 0; i <= 100; ++i) {
        historico_yaw_registar(&h, (int64_t)i * 5000, 0.0f);
    }
    MedidaVisual m = {400000, 0.0f, VISUAL_UNIDADE_GRAUS, 1};
    m.erro_horizontal = 5.0f;
    rastreador_processar_medida(&r, &h, &m, 450000);
    float alvo_ok = r.yaw_alvo;
    m.t_captura_us = 450000; m.erro_horizontal = 45.0f;
    int aceite = rastreador_processar_medida(&r, &h, &m, 480000);
    VERIFICAR(!aceite && fabsf(r.yaw_alvo - alvo_ok) < 1e-6f,
              "salto isolado de 40 graus (reflexo) e ignorado");
    for (int k = 0; k < 3; ++k) {
        m.t_captura_us = 460000 + k * 1000;
        aceite = rastreador_processar_medida(&r, &h, &m, 490000);
    }
    VERIFICAR(aceite && fabsf(GRAUS(r.yaw_alvo) - 45.0f) < 0.01f,
              "salto confirmado em 3 imagens seguidas e aceite");
}

/* ======================= PARTE 2: SIMULACAO ======================= */

/* Modelo fisico (valores a substituir pelos medidos no cubo real). */
#define J_CORPO      0.05    /* kg.m^2 inercia do cubo em yaw */
#define J_RODA       0.001   /* kg.m^2 inercia da roda */
#define ATRITO_VISC  0.002   /* N.m.s/rad rolamentos do gimbal */
#define ATRITO_COUL  0.002   /* N.m atrito seco */
#define SUBPASSO     0.0005  /* s */
#define GIRO_RUIDO   0.003   /* rad/s (1 sigma) */
#define GIRO_VIES    0.0005  /* rad/s vies residual apos calibracao */
#define CAM_PERIODO_US   50000   /* 20 fps */
#define CAM_LATENCIA_US  100000  /* 0,1 s entre captura e resultado */
#define CAM_FOCAL_PX     277.0f
#define CAM_MEIO_FOV     RAD(35.0f)
#define CAM_RUIDO_PX     0.7f
#define MAX_FILA_CAM     16

static uint32_t semente = 12345u;
static double aleatorio_uniforme(void) {
    semente = semente * 1664525u + 1013904223u;
    return ((semente >> 8) + 0.5) / 16777216.0;
}
static double gaussiana(void) {
    double u1 = aleatorio_uniforme(), u2 = aleatorio_uniforme();
    return sqrt(-2.0 * log(u1)) * cos(6.283185307 * u2);
}

typedef struct {
    double yaw, w, w_roda;     /* corpo (rad, rad/s) e roda relativa (rad/s) */
    double torque_ext;         /* perturbacao externa (N.m) */
} Planta;

static void planta_passo(Planta *p, double torque_roda, double dt) {
    double atrito = -ATRITO_VISC * p->w;
    if (p->w > 1e-4) atrito -= ATRITO_COUL;
    else if (p->w < -1e-4) atrito += ATRITO_COUL;
    double alfa_corpo = (-torque_roda + atrito + p->torque_ext) / J_CORPO;
    p->w += alfa_corpo * dt;
    p->w_roda += (torque_roda / J_RODA - alfa_corpo) * dt;
    p->yaw += p->w * dt;
}

typedef struct {
    Planta planta;
    IMU_Filter filtro;
    HistoricoYaw historico;
    RastreadorVisual rastreador;
    int64_t t_us;
    int64_t proxima_captura_us;
    MedidaVisual fila[MAX_FILA_CAM];
    int64_t fila_entrega_us[MAX_FILA_CAM];
    int n_fila;
    double yaw_poster;           /* direcao real do alvo (rad) */
    double vel_poster;           /* rad/s (alvo movel) */
    int camara_tapada;
    int modo_visual;
    float yaw_alvo_gs;
    float torque_pendente;
    EstadoVisual estado_visual;
} Sim;

static void sim_iniciar(Sim *s, int modo_visual, double yaw_poster) {
    s->planta.yaw = 0; s->planta.w = 0; s->planta.w_roda = 0;
    s->planta.torque_ext = 0;
    imu_filter_init(&s->filtro);
    historico_yaw_init(&s->historico);
    rastreador_init(&s->rastreador, CAM_FOCAL_PX, 1);
    s->t_us = 0;
    s->proxima_captura_us = 13000;
    s->n_fila = 0;
    s->yaw_poster = yaw_poster;
    s->vel_poster = 0;
    s->camara_tapada = 0;
    s->modo_visual = modo_visual;
    s->yaw_alvo_gs = 0;
    s->torque_pendente = 0;
    s->estado_visual = VISUAL_SEM_ALVO;
    reset_controladores();
}

/* Um ciclo de controlo (~5 ms) com fisica em subpassos. */
static void sim_ciclo(Sim *s) {
    /* jitter de +-0,2 ms no periodo */
    int64_t periodo = 5000 + (int64_t)((aleatorio_uniforme() - 0.5) * 400.0);
    double dt = periodo * 1e-6;
    int n = (int)(dt / SUBPASSO + 0.5);
    for (int k = 0; k < n; ++k) {
        planta_passo(&s->planta, s->torque_pendente, dt / n);
        s->yaw_poster += s->vel_poster * dt / n;
    }
    s->t_us += periodo;

    /* Camara: captura (instante real) e entrega com latencia. */
    while (s->t_us >= s->proxima_captura_us) {
        double e = remainder(s->yaw_poster - s->planta.yaw, 2 * M_PI);
        MedidaVisual m;
        m.t_captura_us = s->proxima_captura_us;
        m.unidade = VISUAL_UNIDADE_PIXEIS;
        m.alvo_detetado = (!s->camara_tapada && fabs(e) < CAM_MEIO_FOV);
        m.erro_horizontal = m.alvo_detetado ?
            (float)(CAM_FOCAL_PX * tan(e) + CAM_RUIDO_PX * gaussiana()) : 0.0f;
        if (s->n_fila < MAX_FILA_CAM) {
            s->fila[s->n_fila] = m;
            s->fila_entrega_us[s->n_fila] = m.t_captura_us + CAM_LATENCIA_US;
            s->n_fila++;
        }
        s->proxima_captura_us += CAM_PERIODO_US;
    }

    /* IMU */
    float gz = (float)(s->planta.w + GIRO_VIES + GIRO_RUIDO * gaussiana());
    imu_filter_update(&s->filtro, 0, 0, (float)IMU_GRAVITY,
                      (float)(GIRO_RUIDO * gaussiana()),
                      (float)(GIRO_RUIDO * gaussiana()), gz,
                      0, 0, 0, (float)dt);
    historico_yaw_registar(&s->historico, s->t_us, s->filtro.estimativa.atitude.yaw);

    /* Entrega da camara (fila de tamanho 1: so a mais recente conta). */
    int ultima = -1, k = 0;
    for (int i = 0; i < s->n_fila; ++i) {
        if (s->fila_entrega_us[i] <= s->t_us) ultima = i;
    }
    if (ultima >= 0) {
        rastreador_processar_medida(&s->rastreador, &s->historico,
                                    &s->fila[ultima], s->t_us);
        for (int i = ultima + 1; i < s->n_fila; ++i, ++k) {
            s->fila[k] = s->fila[i];
            s->fila_entrega_us[k] = s->fila_entrega_us[i];
        }
        s->n_fila = k;
    }
    float yaw_alvo_vis = s->filtro.estimativa.atitude.yaw;
    s->estado_visual = rastreador_obter_alvo(&s->rastreador, s->t_us,
                                             s->filtro.estimativa.atitude.yaw,
                                             &yaw_alvo_vis);

    Attitude3 alvo = {0, 0, s->modo_visual ? yaw_alvo_vis : s->yaw_alvo_gs};
    Attitude3 vel_rodas = {NAN, NAN, (float)s->planta.w_roda};
    WheelCommands c;
    if (calcular_comandos_rodas(&s->filtro.estimativa, &alvo, &vel_rodas,
                                (float)dt, &c)) {
        s->torque_pendente = c.yaw.torque_roda;
    } else {
        s->torque_pendente = 0.0f;
    }
}

static double erro_real_graus(const Sim *s) {
    return remainder(s->yaw_poster - s->planta.yaw, 2 * M_PI) * 57.29577951;
}

static void ensaio_degrau_45(void) {
    Sim s;
    sim_iniciar(&s, 0, 0);
    s.yaw_alvo_gs = RAD(45.0f);
    double pico = 0, t_assent = -1;
    for (int i = 0; i < 1600; ++i) {   /* 8 s */
        sim_ciclo(&s);
        double y = s.planta.yaw * 57.29577951;
        if (y > pico) pico = y;
        if (fabs(y - 45.0) > 1.0) t_assent = -1;
        else if (t_assent < 0) t_assent = s.t_us * 1e-6;
    }
    double final = s.planta.yaw * 57.29577951;
    printf("  degrau 45: final %.2f graus, pico %.2f graus, assenta (+-1) aos %.2f s\n",
           final, pico, t_assent);
    VERIFICAR(fabs(final - 45.0) < 1.0 && t_assent > 0,
              "Nivel 3: degrau de 45 graus com erro final < 1 grau");
}

static void ensaios_visuais(void) {
    Sim s;
    /* ---- Ensaio 1: aquisicao a 30 graus ---- */
    sim_iniciar(&s, 1, RAD(30.0));
    double t_centrado = -1;
    for (int i = 0; i < 2000; ++i) {   /* 10 s */
        sim_ciclo(&s);
        if (fabs(erro_real_graus(&s)) > 2.0) t_centrado = -1;
        else if (t_centrado < 0) t_centrado = s.t_us * 1e-6;
    }
    printf("  aquisicao: centrado (+-2) aos %.2f s; erro aos 10 s = %.2f graus\n",
           t_centrado, erro_real_graus(&s));
    VERIFICAR(t_centrado > 0 && t_centrado < 6.0,
              "Ensaio 1 / Nivel 5: de 30 graus ao centro em < 6 s");

    /* ---- Ensaio 2: hold 60 s ---- */
    double max_hold = 0;
    for (int i = 0; i < 12000; ++i) {
        sim_ciclo(&s);
        double e = fabs(erro_real_graus(&s));
        if (e > max_hold) max_hold = e;
    }
    printf("  hold: erro maximo em 60 s = %.2f graus (aceites %u, rejeitadas %u)\n",
           max_hold, (unsigned)s.rastreador.imagens_aceites,
           (unsigned)s.rastreador.imagens_rejeitadas);
    VERIFICAR(max_hold < 2.0, "Ensaio 2: hold de 60 s dentro de +-2 graus");

    /* ---- Ensaio 3: perturbacao (~20 graus) ---- */
    double max_pert = 0, t_inicio = s.t_us * 1e-6, t_rec = -1;
    for (int i = 0; i < 1600; ++i) {   /* 8 s */
        s.planta.torque_ext = (i < 40) ? 0.5 : 0.0;   /* empurrao de 0,2 s */
        sim_ciclo(&s);
        double e = fabs(erro_real_graus(&s));
        if (e > max_pert) max_pert = e;
        if (i >= 40) {
            if (e > 2.0) t_rec = -1;
            else if (t_rec < 0) t_rec = s.t_us * 1e-6 - t_inicio;
        }
    }
    printf("  perturbacao: desvio maximo %.1f graus, recupera (+-2) em %.2f s\n",
           max_pert, t_rec);
    VERIFICAR(max_pert > 10.0 && t_rec > 0 && t_rec < 4.0,
              "Ensaio 3: recupera de um empurrao em < 4 s");

    /* ---- Ensaio 5: oclusao de 2 s ---- */
    double max_ocl = 0;
    int viu_oclusao = 0;
    s.camara_tapada = 1;
    for (int i = 0; i < 400; ++i) {
        sim_ciclo(&s);
        double e = fabs(erro_real_graus(&s));
        if (e > max_ocl) max_ocl = e;
        if (s.estado_visual == VISUAL_OCLUSAO) viu_oclusao = 1;
    }
    s.camara_tapada = 0;
    for (int i = 0; i < 400; ++i) {
        sim_ciclo(&s);
    }
    printf("  oclusao: erro maximo durante 2 s tapada = %.2f graus, estado final = %d\n",
           max_ocl, (int)s.estado_visual);
    VERIFICAR(viu_oclusao && max_ocl < 2.0 && s.estado_visual == VISUAL_A_SEGUIR,
              "Ensaio 5: camara tapada 2 s, nao perde a orientacao e retoma");

    /* ---- Ensaio 4: alvo movel (5 graus/s durante 8 s) ---- */
    double max_mov = 0;
    int perdeu = 0;
    s.vel_poster = RAD(5.0);
    for (int i = 0; i < 1600; ++i) {
        sim_ciclo(&s);
        double e = fabs(erro_real_graus(&s));
        if (i > 200 && e > max_mov) max_mov = e;
        if (s.estado_visual != VISUAL_A_SEGUIR) perdeu = 1;
    }
    printf("  alvo movel a 5 graus/s: erro maximo (apos 1 s) = %.2f graus\n", max_mov);
    /* Criterio do enunciado: segue sem perder (o ponto nunca sai da imagem).
     * O atraso de ~3 graus vem de Kp da malha externa (5 graus/s / Kp). */
    VERIFICAR(!perdeu && max_mov < 10.0,
              "Ensaio 4: segue o alvo movel sem o perder (sempre na imagem)");
}

int main(void) {
    printf("=== V5 — testes unitarios ===\n");
    if (!init_controladores()) {
        printf("FALHOU: init_controladores\n");
        return EXIT_FAILURE;
    }
    teste_filtro_imu();
    teste_compensacao_inclinacao();
    teste_pid_dt();
    teste_pid_derivada();
    teste_anti_windup();
    teste_saturacao_roda();
    teste_visual_unitario();

    printf("\n=== V5 — simulacao (200 Hz, camara 20 fps com 100 ms de atraso) ===\n");
    init_controladores();
    ensaio_degrau_45();
    ensaios_visuais();

    printf("\n%s (%d falha(s))\n", falhas_totais ? "RESULTADO: FALHOU" :
           "RESULTADO: TODOS OS TESTES PASSARAM", falhas_totais);
    return falhas_totais ? EXIT_FAILURE : EXIT_SUCCESS;
}
