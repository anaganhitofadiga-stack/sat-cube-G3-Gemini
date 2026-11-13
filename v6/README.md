# Controlo de atitude — V5 ("Cubo que aprende a apontar")

Controlo do eixo **yaw** com roda de reação (Moteus r4.11 + mj5208 via CAN-FD), estimativa de atitude com a
MPU-6050 e **alvo de yaw vindo da câmara** (Níveis 4–5 / WBS 4.3). Roll e pitch só têm amortecimento.
Unidades: rad, rad/s, s, N.m. As diferenças face à V4 estão em `CHANGELOG.md`.

## Arquitetura

```
 Core 0 (camara + Wi-Fi)                         Core 1 (controlo, 200 Hz)
 ┌──────────────────────┐   MedidaVisual   ┌──────────────────────────────────────────────┐
 │ OV5640 → ponto branco │ ──fila (1)────▶ │ IMU → filtro → historico yaw                 │
 │ t_captura + erro px   │                 │ rastreador visual → yaw alvo                 │
 └──────────────────────┘                 │ PID posicao → PID velocidade → protecao roda │──CAN-FD──▶ moteus
 ┌──────────────────────┐ telecomandos    │                                               │
 │ TMTC / Ground Station │ ◀──telemetria── │                                               │
 └──────────────────────┘                 └──────────────────────────────────────────────┘
```

| Ficheiro | Corre em | Função |
|---|---|---|
| `controlo_atitude.c/.h` | PC e ESP32 | PID, cascata, amortecimento, proteção da roda, filtro IMU |
| `controlo_visual.c/.h` | PC e ESP32 | histórico de yaw, conversão do erro visual, rastreador |
| `hal_cubo.h` | — | contrato com os drivers (implementado pelo Edson em `hal_cubo.c`) |
| `app_controlo.c/.h` | só ESP32 | tarefas FreeRTOS, filas, segurança, telemetria |
| `teste.c` | só PC | testes unitários + simulação dos 5 ensaios |

## Compilar e testar no PC (MSYS2 MINGW64)

```bash
gcc -Wall -Wextra -O2 -o teste teste.c controlo_atitude.c controlo_visual.c -lm
./teste
```

## Integrar no ESP-IDF

1. Copiar `controlo_atitude.*`, `controlo_visual.*`, `hal_cubo.h`, `app_controlo.*` para `main/` e
   acrescentá-los ao `SRCS` do `CMakeLists.txt`.
2. Implementar `hal_cubo.c` (Edson) respeitando as convenções de eixos, unidades e sinais de `hal_cubo.h`.
3. No `app_main()`: inicializar I2C, SPI, MCP2518FD e câmara; depois `app_controlo_iniciar()`.
4. Na tarefa da câmara (Ricardo, Core 0): preencher `MedidaVisual` com o **instante da captura** e chamar
   `app_controlo_publicar_medida_visual()` (exemplo no fim de `app_controlo.c`).
5. Na tarefa TMTC (Alessia/Ricardo): `app_controlo_enviar_telecomando()` e `app_controlo_ler_telemetria()`.
6. No moteus, configurar também os limites internos (corrente máxima, velocidade máxima, *watchdog*) como
   segunda barreira, independente do ESP32.

## Primeiro ensaio no hardware (ordem recomendada)

1. **Sinal do motor:** modo `MODO_ATITUDE_GS`, roda desacoplada, binário pequeno → confirmar que binário positivo
   faz o corpo girar em −z. Se não, inverter o sinal em `hal_moteus_comandar_binario`.
2. **Sinal do giroscópio:** rodar o cubo à mão em +z → `wz` positivo na telemetria.
3. **Degrau 45° (Nível 3)** com ganhos de posição reduzidos a metade; subir aos poucos.
4. **Sinal da câmara:** alvo à direita da imagem → o cubo deve rodar para ele. Se fugir, `CAMERA_SINAL = -1`.
5. **Distância focal:** medir `CAMERA_DISTANCIA_FOCAL_PX` com a resolução final (ver `app_controlo.h`).
6. Só depois `MODO_VISUAL` e os 5 ensaios.

## Afinação

- Resposta lenta → subir `KP_POS_YAW`. Oscilação → descer `KP_POS_YAW` ou subir `KD_POS_YAW`.
- Alvo móvel com atraso grande → subir `KI_POS_YAW` com cuidado (atraso ≈ velocidade do alvo / `KP_POS_YAW`).
- Erro parado por atrito → subir `KI_VEL_YAW`.
- Muitas imagens rejeitadas (telemetria `imagens_rejeitadas`) → medir a latência real e ajustar `VISUAL_IDADE_MAX_S`.

## Limitações conhecidas

- O filtro integra as taxas do giroscópio como taxas de Euler (válido com o cubo pouco inclinado).
- Sem magnetómetro, o yaw da IMU deriva lentamente; só a câmara o corrige (o alvo é guardado no mesmo referencial
  da IMU, por isso deriva lenta não afeta o apontamento enquanto houver imagens).
- O ponto tem de estar dentro do campo de visão no arranque; não há modo de procura.
- Os parâmetros físicos do simulador (inércias, atrito) são estimativas.
