# Changelog — V4 → V5

Data: 11 Nov 2026 · Autora: Ana (Simulação, Dinâmica de Atitude e Gestão de Integração)
Âmbito: WBS 4.1 / 4.3 — Níveis 4 e 5 (erro visual na malha de controlo) + revisão de segurança da V4.

## Ficheiros

| Ficheiro | Estado | Conteúdo |
|---|---|---|
| `controlo_atitude.h/.c` | alterado | PID, cascata yaw, amortecimento roll/pitch, proteção da roda, filtro IMU |
| `controlo_visual.h/.c` | **novo** | histórico de yaw, conversão píxeis/graus → yaw, rastreador com oclusão e rejeição de falsos alvos |
| `hal_cubo.h` | **novo** | contrato com os drivers do Edson (IMU, magnetómetro, moteus, relógio) |
| `app_controlo.h/.c` | **novo** | integração FreeRTOS: tarefa de 200 Hz no Core 1, filas para câmara e Ground Station, segurança |
| `teste.c` | reescrito | 22 testes unitários + simulação dos 5 ensaios do enunciado |
| `README.md` | atualizado | como compilar, integrar e afinar |

## Correções à V4

1. **Pico de derivada quando o alvo muda (bug real com a câmara).**
   V4: a derivada da malha externa era calculada sobre o *erro*. Cada imagem nova (15–20 por segundo) muda o alvo e
   o erro salta → derivada enorme → comando saturado a cada imagem. No teste, um salto de 1 rad dava 21 em vez de 1.
   V5: a derivada é calculada sobre a *medida* (yaw do cubo), que nunca salta, e tem filtro passa-baixo (`TAU_D_POS_YAW`).

2. **Derivada errada ao cruzar ±180°.** Com a derivada na medida, a variação do yaw é envolvida para [−π, π]
   antes de dividir por dt; caso contrário, passar de +179° para −179° pareceria uma rotação de 358°.

3. **Proteção do dt incompleta.** V4 aceitava qualquer `dt > 0`; um dt de 1e-9 s (dois tiques seguidos) dividia
   por quase zero na derivada, e um dt de vários segundos (tarefa bloqueada) fazia o integral saltar.
   V5: `dt < DT_MIN_S` (0,5 ms) → ciclo ignorado; `dt > DT_MAX_S` (50 ms) → dt limitado e derivada reiniciada.

4. **Anti-windup sem ligação entre malhas.** V4 tinha anti-windup em cada PID mas a malha externa não sabia se a
   interna estava saturada: o integral de posição continuava a crescer enquanto o motor já dava tudo.
   V5: se a malha interna ou a roda estiverem no limite, o integral da malha externa pode diminuir mas não crescer
   (`congelar_integral`). Além disso, quando a integração é recusada, a saída passa a ser recalculada com o integral
   realmente guardado (na V4 usava o provisório).

5. **Sem proteção de velocidade da roda.** O enunciado avisa que "a roda tem limite". V4 só limitava o binário.
   V5: `proteger_saturacao_roda()` reduz progressivamente o binário que acelera a roda acima de 2400 rpm e anula-o
   aos 3000 rpm (o binário que trava a roda passa sempre). A tarefa FreeRTOS para o motor acima de 3600 rpm.

6. **Ângulos ligeiramente fora de [−π, π] desligavam tudo.** V4 rejeitava o ciclo inteiro (incluindo o
   amortecimento) se o yaw viesse com 3,1416. V5 envolve os ângulos em vez de rejeitar.

7. **Filtro da IMU exigia magnetómetro.** O hardware do projeto tem a MPU-6050 e o magnetómetro fica a centímetros
   de um motor de 600 W. Na V4, magnetómetro a zeros ⇒ filtro falhava ⇒ nenhum controlo. V5: magnetómetro opcional
   (zeros = desligado); o yaw vem do giroscópio e a câmara corrige a deriva.

8. **Filtro complementar dependente da frequência.** `alpha = 0,98` por amostra corrige 2× mais depressa a 200 Hz
   do que a 100 Hz. V5 usa uma constante de tempo (`IMU_COMPLEMENTARY_TAU` = 0,49 s, igual a 0,98 a 100 Hz).

9. **Idade máxima da imagem igual à latência esperada.** V4 tinha `MAX_VISUAL_AGE_S = 0,1 s` e o enunciado diz que a
   imagem chega com quase 0,1 s de atraso → muitas imagens seriam descartadas. V5: 0,25 s (o atraso é compensado
   pelo histórico de yaw, por isso uma imagem de 0,2 s continua correta).

10. **Limite de passo do giroscópio mais realista.** Um salto de mais de 180° numa só amostra é fisicamente
    impossível; a V5 rejeita a amostra a partir de π (V4: 2π).

## Funcionalidades novas (Níveis 4–5)

- **Histórico de yaw** (`HistoricoYaw`): guarda 0,64 s de yaw e devolve, por interpolação, o yaw no instante exato
  de cada fotografia. Resolve o problema "a imagem chega atrasada".
- **Rastreador visual** (`RastreadorVisual`): estados `SEM_ALVO` / `A_SEGUIR` / `OCLUSAO`; aceita erro em píxeis
  *ou* graus; segura o último alvo com a IMU quando a câmara é tapada (ensaio 5); ignora saltos > 15° que não se
  repitam em 3 imagens (reflexos, ensaio de luz variável); suporta *offset* para o nível 7.
- **Integração FreeRTOS** (`app_controlo.c`): temporizador de 5 ms, tarefa fixa no Core 1, filas de tamanho 1 com
  `xQueueOverwrite` (só a imagem mais recente interessa), calibração do viés do giroscópio no arranque, armar /
  desarmar / paragem de emergência, deteção de falhas da IMU, do moteus e de ciclos atrasados, telemetria a 20 Hz.

## Alterações de interface (quem chama estas funções tem de ser atualizado)

| V4 | V5 |
|---|---|
| `calcular_pid(pid, erro, dt, min, max)` | `calcular_pid(pid, erro, medida, dt, min, max, congelar_integral)` |
| `calcular_malha_externa_yaw(erro, dt)` | `calcular_malha_externa_yaw(erro, yaw_medido, dt, congelar)` |
| `calcular_malha_intermedia_yaw(vd, vr, dt)` | `calcular_malha_intermedia_yaw(vd, vr, dt, congelar)` |
| `calcular_comandos_rodas(estado, alvo, dt, cmd)` | `calcular_comandos_rodas(estado, alvo, vel_rodas, dt, cmd)` |
| `atualizar_alvo_yaw_visual` em `controlo_atitude` | passou para `controlo_visual` (mesma assinatura) |
| `MAX_VISUAL_AGE_S` (0,1 s) | `VISUAL_IDADE_MAX_S` (0,25 s) |
| `IMU_COMPLEMENTARY_ALPHA` | `IMU_COMPLEMENTARY_TAU` |
| `WheelCommand` | campo novo `limitado` |

## Resultados da simulação (SIL, 200 Hz, câmara 20 fps com 100 ms de atraso)

| Ensaio | Critério | V5 |
|---|---|---|
| Nível 3 — degrau 45° | erro final < ±1° | 45,04°, assenta em 2,0 s, sem sobreelevação |
| 1 — Aquisição 30° | < 6 s | 1,7 s |
| 2 — Hold 60 s | ±2° | máx. 0,28° |
| 3 — Perturbação ~20° | < 4 s | desvio 22°, recupera em 1,9 s |
| 4 — Alvo móvel (5°/s) | não perder | atraso máx. 3,6°, nunca perdido |
| 5 — Oclusão 2 s | não perder orientação | máx. 0,20°, retoma sozinho |

Também passa com inércia do cubo de 0,02 a 0,1 kg·m² e atraso da câmara até 200 ms.
**A simulação não substitui os testes de bancada**: atrito real, folgas, ruído e sinais têm de ser verificados.
