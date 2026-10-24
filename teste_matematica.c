#include <stdio.h>
#include "controlo_atitude.h"

int main() {
    // 1. Inicializa o controlador
    init_controladores();
    
    // 2. Simula uma situação: O alvo visual está a 15 graus de distância
    float erro_posicao_graus = 15.0; 
    float vel_real_giroscopio = 0.0; // Cubo está parado neste momento
    float dt = 0.05; // Delta time de 50ms (simulando 20 frames por segundo)
    
    printf("--- INICIO DO TESTE UNITARIO ---\n");
    printf("Erro inicial do Cubo: %.2f graus\n", erro_posicao_graus);
    
    // 3. Calcula a Malha Externa (Que velocidade angular o cubo precisa?)
    float vel_cubo_desejada = calcular_malha_externa_yaw(erro_posicao_graus, dt);
    printf("1. Malha Externa -> Velocidade pedida ao Cubo: %.2f graus/s\n", vel_cubo_desejada);
    
    // 4. Calcula a Malha Intermédia (Que comando dar ao motor para atingir essa velocidade?)
    float comando_motor = calcular_malha_intermedia_yaw(vel_cubo_desejada, vel_real_giroscopio, dt);
    printf("2. Malha Interna -> Comando enviado para a Roda: %.2f RPM\n", comando_motor);
    
    printf("--------------------------------\n");
    
    return 0;
}