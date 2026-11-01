#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "controlo_atitude.h"

/*
 * Ensaio de simulação: verifica o amortecimento nos eixos passivos e aplica
 * um degrau de 45 graus à cascata de yaw durante aproximadamente 3 segundos.
 */
int main(void) {
    /* Carrega os ganhos e zera a memória dos controladores antes do ensaio. */
    init_controladores();

    /* Fatores para converter ângulos de/para radianos, unidade usada no controlador. */
    const float radianos_por_grau = 0.01745329252f;
    const float graus_por_radiano = 57.29577951f;

    /* Sequência repetitiva que simula variação do período de amostragem (48–52 ms). */
    const float dt_amostras[] = {
        0.048f, 0.052f, 0.049f, 0.051f, 0.050f, 0.052f, 0.048f
    };
    /* Número de amostras da sequência; permite percorrê-la com módulo. */
    const int numero_intervalos_dt =
        (int)(sizeof(dt_amostras) / sizeof(dt_amostras[0]));

    /* Parâmetros e estado do modelo físico simplificado do ensaio. */
    const float inercia_cubo = 0.05f;  // Inércia fictícia usada para calcular aceleração.
    const float atitude_alvo = 45.0f * radianos_por_grau;  // Degrau de referência em radianos.
    const int numero_passos = 60;     // Quantidade de ciclos simulados.
    float atitude_cubo = 0.0f;        // Posição simulada do corpo (rad).
    float vel_cubo = 0.0f;            // Velocidade simulada do corpo (rad/s).
    float tempo_atual = 0.0f;         // Tempo acumulado usando os dt de cada ciclo (s).

    /* Testa se ambos os amortecedores travam o eixo e respeitam o limite de torque. */
    float damping_roll = calcular_amortecimento_roll(1.0f);
    float damping_pitch = calcular_amortecimento_pitch(1.0f);
    if (damping_roll <= 0.0f || damping_pitch <= 0.0f ||
        fabsf(calcular_amortecimento_roll(100.0f) - MAX_TORQUE_COMMAND) > 1e-6f ||
        fabsf(calcular_amortecimento_pitch(100.0f) - MAX_TORQUE_COMMAND) > 1e-6f) {
        fprintf(stderr, "FALHOU: verificacao do amortecimento de roll/pitch.\n");
        return EXIT_FAILURE;
    }

    /* Cabeçalho da telemetria impressa durante a resposta ao degrau. */
    printf("--- TESTE DE RESPOSTA AO DEGRAU (45 GRAUS) ---\n");
    printf("dt variavel entre 48 e 52 ms\n");
    printf("Tempo(s) | dt(ms) | Erro(graus) | Vel. cubo(graus/s) | Binario roda(N.m)\n");
    printf("-------------------------------------------------------------------------\n");

    /* Executa os ciclos de leitura, controlo e atualização do modelo físico. */
    for (int i = 0; i < numero_passos; i++) {
        /* Reutiliza a sequência para representar pequenas variações de relógio. */
        float dt = dt_amostras[i % numero_intervalos_dt];

        /* Erro entre o setpoint de 45 graus e a atitude simulada atual (rad). */
        float erro_posicao = atitude_alvo - atitude_cubo;

        /* Cascata: posição gera velocidade desejada; velocidade gera comando de roda. */
        float vel_desejada = calcular_malha_externa_yaw(erro_posicao, dt);
        float comando_roda = calcular_malha_intermedia_yaw(
            vel_desejada, vel_cubo, dt);

        /* Imprime um ponto a cada cinco ciclos para acompanhar a resposta sem excesso de linhas. */
        if (i % 5 == 0) {
            printf("%4.2fs   | %6.1f | %10.2f  | %18.2f | %17.3f\n",
                   tempo_atual,
                   dt * 1000.0f,
                   erro_posicao * graus_por_radiano,
                   vel_cubo * graus_por_radiano,
                   comando_roda);
        }

        /* O atuador nunca deve ultrapassar o limite definido para a roda. */
        if (fabsf(comando_roda) > MAX_TORQUE_COMMAND) {
            fprintf(stderr, "O binario comandado excedeu o limite de seguranca.\n");
            return EXIT_FAILURE;
        }

        /*
         * Modelo simplificado: a roda exerce no corpo um binário de reação
         * oposto ao binário comandado à roda; aceleração = binário / inércia.
         */
        float aceleracao = -comando_roda / inercia_cubo;

        /* Integra aceleração para obter velocidade e, em seguida, atitude do corpo. */
        vel_cubo += aceleracao * dt;
        atitude_cubo += vel_cubo * dt;
        tempo_atual += dt;  // Soma o intervalo real desta iteração ao tempo decorrido.
    }

    /* Calcula e apresenta o erro residual no fim da simulação, em graus. */
    float erro_final_graus = (atitude_alvo - atitude_cubo) * graus_por_radiano;
    printf("Erro final aos %.2fs: %.3f graus\n", tempo_atual, erro_final_graus);

    /* Critério de aceitação do ensaio: erro finito e inferior ou igual a 1 grau. */
    if (!isfinite(erro_final_graus) || fabsf(erro_final_graus) > 1.0f) {
        fprintf(stderr, "FALHOU: erro final superior a +/-1 grau.\n");
        return EXIT_FAILURE;
    }

    printf("PASSOU: erro final dentro do limite de +/-1 grau.\n");
    return EXIT_SUCCESS;
}