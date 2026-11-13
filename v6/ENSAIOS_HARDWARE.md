# Testar o controlo de atitude no cubo real (cabo USB)

Guia de bancada para o firmware V5 no **Seeed Studio XIAO ESP32-S3 Sense**.
O código de controlo (`controlo_atitude.c`, `controlo_visual.c`, `app_controlo.c`)
não muda: o que foi acrescentado é a camada de hardware e a ponte USB.

```
v5/
├── firmware/                  projeto ESP-IDF (compila o código da raiz)
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults     consola em USB Serial JTAG, 8 MB, tique 1 ms
│   └── main/
│       ├── main.c             app_main()
│       ├── placa_xiao.h       mapa de pinos do XIAO
│       ├── hal_cubo.c         MPU-6050 + LIS3MDL + relógio + moteus
│       ├── hal_cubo_hw.h      arranque e diagnóstico do hardware
│       ├── app_usb.c/.h       telecomandos e telemetria CSV pelo USB
│       ├── drv_moteus.c/.h    protocolo CAN-FD do moteus
│       └── (drv_mcp2518fd.c)  FALTA — transporte SPI/CAN-FD (Edson)
└── ferramentas/
    ├── gs_cubo.py             registo CSV + gráficos no PC
    └── teste_moteus.c         teste do protocolo CAN-FD, sem hardware
```

O `teste_moteus.c` corre no PC e confirma byte a byte que as tramas enviadas ao
moteus estão conformes a documentação do mjbots (26 verificações, incluindo o
ID `0x8001`, os registos escritos, o enchimento com *nop* e a conversão de
Hz para rad/s). Compilar no MSYS2 MINGW64:

```bash
cd ferramentas
gcc -Wall -Wextra -I../firmware/main -o teste_moteus teste_moteus.c ../firmware/main/drv_moteus.c -lm
./teste_moteus
```

## O que o cabo USB faz e não faz

O USB do XIAO vai direto ao USB nativo do ESP32-S3 (não há chip USB-série).
Serve para **gravar o firmware, ver os registos, enviar telecomandos e receber
telemetria**. Não alimenta o motor: o moteus + mj5208 precisam de fonte DC
própria.

Por isso o firmware arranca em **modo de bancada**
(`HAL_MOTEUS_MODO = 0` em `hal_cubo.c`): o binário é calculado e aparece na
telemetria, mas **não é enviado ao motor**. Isto permite validar hoje, só com o
cabo, tudo menos o atuador: I²C, MPU-6050, filtro, estimativa de yaw,
calibração do viés do giroscópio, cascata PID, proteções, telecomandos,
telemetria e temporização a 200 Hz.

## Material

Para a fase de bancada (passo 0), basta:

- XIAO ESP32-S3 Sense + cabo USB-C de dados
- MPU-6050 montada **rigidamente** no cubo, eixos como em `hal_cubo.h`
  (x = roll, y = pitch, z = yaw, z para cima)
- PC com ESP-IDF e Python

Para fechar a malha no motor, acresce:

- moteus r4.11 + mj5208 com a roda de reação montada e equilibrada
- MCP2518FD ligado por SPI ao XIAO e por CAN-FD ao moteus, com terminação
  de **120 Ω**
- fonte DC de bancada para o moteus, com **limite de corrente** (começar baixo)
- **interruptor de corte** da alimentação do motor, ao alcance da mão
- suspensão de baixo atrito: fio de torção fino, rolamento ou mesa de ar.
  Apoiado numa mesa, o atrito domina e os ensaios não dizem nada
- magnetómetro LIS3MDL (opcional, e desligado por omissão: fica a centímetros
  de um motor de 600 W)
- `drv_mcp2518fd.c` implementado

> **Confirmar com o Nuno Cruz:** o moteus exige uma **calibração do motor feita
> uma vez** a partir do PC (`moteus_tool --calibrate`), normalmente com um
> adaptador CAN-FD tipo `fdcanusb`. Sem essa calibração o mj5208 não responde a
> comandos de binário, por mais correto que esteja o firmware. Vale a pena
> perguntar se o laboratório tem o adaptador antes de contar com o motor.

## Ligações

| XIAO | GPIO | Para |
|---|---|---|
| D4 | 5 | SDA — MPU-6050 (+ LIS3MDL) |
| D5 | 6 | SCL — MPU-6050 (+ LIS3MDL) |
| D2 | 3 | MCP2518FD CS (pull-up 10 kΩ para 3V3) |
| D3 | 4 | MCP2518FD INT |
| D8 | 7 | SPI SCK |
| D9 | 8 | SPI MISO |
| D10 | 9 | SPI MOSI |
| D1 | 2 | buzzer |
| D0 | 1 | tensão da bateria (divisor 100 k / 13 k) |

A MPU-6050 alimenta-se a **3V3** do XIAO. Se a placa da IMU for das de 5 V com
resistências de *pull-up* para 5 V, não ligar o I²C diretamente: as entradas do
ESP32-S3 são de 3,3 V. Confirmar com o multímetro antes de ligar.

## Compilar e gravar

Atalho: **duplo clique em `ligar_cubo.bat`**, na raiz do `v5`. Procura o
ESP-IDF, descobre a porta do XIAO, compila, grava e abre logo a ligação,
gravando um CSV com data e hora no nome. Para forçar a porta:
`ligar_cubo.bat COM7`. Se o script não encontrar o ESP-IDF, abre o terminal do
ESP-IDF no VS Code (`Ctrl+Shift+P` → *ESP-IDF: Open ESP-IDF Terminal*) e corre
`ligar_cubo.bat` de lá.

Os passos à mão, para quando algo falhar:

No terminal do ESP-IDF (ou no painel da extensão do VS Code):

```bash
cd firmware
idf.py set-target esp32s3
idf.py build
idf.py -p COM7            flash          # trocar COM7 pela porta do XIAO
```

Se o XIAO não aparecer como porta: manter **BOOT** premido, dar um toque em
**RESET**, largar o BOOT e gravar outra vez.

Depois de gravar, usar **uma** das duas vias para falar com o cubo (a porta só
aceita um programa de cada vez):

```bash
idf.py -p COM7 monitor                              # só ver (Ctrl+] para sair)
python ../ferramentas/gs_cubo.py --porta COM7 --saida ensaio1.csv
```

O `gs_cubo.py` é o preferível nos ensaios: grava tudo em CSV e aceita os mesmos
comandos. No fim, os gráficos:

```bash
python ../ferramentas/gs_cubo.py --grafico ensaio1.csv
```

## Comandos da consola

| | |
|---|---|
| `a` / `d` | armar / desarmar |
| `x` | paragem de emergência (só sai com `R` ou reset) |
| `m 0,1,2` | modo: parado, atitude pela consola, visual |
| `y <graus>` | yaw alvo absoluto |
| `s <graus>` | degrau relativo ao yaw atual (o mais usado nos ensaios) |
| `z` | yaw alvo = yaw atual |
| `t` | ligar/desligar a telemetria |
| `i` | estado do hardware (erros I²C, erros CAN, falha do moteus) |
| `R` | reiniciar a placa |
| `?` | ajuda |

**O cubo tem de estar parado nos primeiros 2 s depois do arranque**, senão a
calibração do viés do giroscópio falha, fica `FALHA_CALIBRACAO` (16) e o `a`
não arma. Nesse caso: `R` e repetir.

## Procedimento

### Passo 0 — Sem alimentação do motor

Valida a estimativa de atitude e os sinais, que é onde moram quase todos os
erros de montagem.

1. **Arranque.** No registo tem de aparecer `WHO_AM_I da IMU = 0x68` (ou 0x70 /
   0x72 num clone) e `vies giro: ... rad/s` com valores pequenos (tipicamente
   abaixo de 0,05).
2. **Período do ciclo.** `dt_ms` deve estar em 5,00 ± 0,2 e `falhas` a 0.
   Se a calibração demorar 20 s em vez de 2 s, o temporizador não arrancou.
3. **Sinal do giroscópio.** Rodar o cubo à mão no sentido **+z** (anti-horário
   visto de cima): `wz_dps` tem de ficar **positivo** e `yaw_deg` a crescer.
   Se for ao contrário, trocar `HAL_IMU_Z_SINAL` para `-1` em `hal_cubo.c`.
4. **Acelerómetro.** Inclinar para o lado: `roll_deg` acompanha; inclinar para
   a frente: `pitch_deg` acompanha. Com o cubo nivelado, ambos perto de 0.
   Se roll e pitch estiverem trocados, corrigir `HAL_IMU_X_FONTE` /
   `HAL_IMU_Y_FONTE`.
5. **Sinal da malha de controlo** (ainda sem motor). `a`, `m 1`, `z`, e depois
   rodar o cubo à mão **+10° em yaw**. O `torque_Nm` da telemetria tem de ficar
   **positivo**: é o binário na roda que, pela reação, faria o corpo voltar em
   −z. Se der negativo, há um sinal invertido na cadeia — resolver aqui, antes
   de ligar o motor.
6. **Deriva do yaw.** Com o cubo imóvel, ver quanto anda o `yaw_deg` em 60 s.
   Sem magnetómetro, alguns graus por minuto são normais; só a câmara corrige.
   Serve para saber qual a duração máxima útil de um ensaio em `m 1`.

### Passo 1 — Primeiro contacto com o motor

Só depois de `drv_mcp2518fd.c` estar feito e de `HAL_MOTEUS_MODO` passar a 1.

1. Configurar primeiro os limites **dentro do moteus** (corrente máxima,
   velocidade máxima, *watchdog*): é a barreira independente do ESP32, como
   recomenda o `README.md`.
2. Fonte com limite de corrente baixo, roda **desacoplada** (ou cubo travado),
   mão no interruptor.
3. `i` deve mostrar `motor_ativo=1`, `erros_can=0` e `falha_moteus=0`.
4. `a`, `m 1`, `s 5` (degrau pequeno): confirmar que um binário positivo
   acelera a roda em **+z** e que o corpo reage em **−z**. Se não, inverter
   `MOTEUS_SINAL` em `drv_moteus.h`.

### Passo 2 — Ensaios de falha (antes de qualquer ensaio a sério)

Cada um destes tem de parar o motor:

- desligar o cabo CAN → `falhas` com o bit 2 (`FALHA_MOTEUS`) após 100 ms
- desligar a IMU → bit 1 (`FALHA_IMU`) após 50 ms
- `x` → bit 32 (`FALHA_EMERGENCIA`), e o `a` deixa de armar até `R`
- desligar o cabo USB com o cubo armado → o *watchdog* do moteus (0,1 s) corta
  a corrente

### Passo 3 — Degrau de 45° (Nível 3)

Com os ganhos de posição a metade: `KP_POS_YAW` de 2,0 para 1,0 em
`controlo_atitude.h`. Cubo suspenso e livre.

```
a        m 1        z        s 45
```

Critério: erro final abaixo de ±1°. Na simulação assenta em 2,0 s sem
sobreelevação (ver `CHANGELOG.md`). Registar o CSV, desenhar e comparar. Subir
`KP_POS_YAW` aos poucos até à resposta da simulação; a secção "Afinação" do
`README.md` diz o que mexer em cada sintoma.

Vigiar `roda_rpm`: a redução progressiva do binário começa aos 2400 rpm, a
proteção anula aos 3000 e a tarefa desarma aos 3600. Se a roda saturar muito
cedo, a inércia da roda é pequena demais para o cubo ou os ganhos estão altos.

### Passo 4 — Ensaios 1 a 5

Só depois do degrau limpo. A câmara entra agora: medir
`CAMERA_DISTANCIA_FOCAL_PX` na resolução final, confirmar `CAMERA_SINAL`
(alvo à direita → o cubo roda para ele; se fugir, pôr `-1`), e seguir a tabela
de critérios do `CHANGELOG.md`.

## Se algo não corre bem

| Sintoma | Causa provável |
|---|---|
| Porta não aparece | cabo sem dados, ou falta o BOOT+RESET |
| Monitor em branco | falta `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` (apagar `sdkconfig` e recompilar) |
| `FALHA_IMU` logo no início | endereço I²C (AD0 → 0x69), alimentação, ou *pull-ups* a 5 V |
| `erros_i2c` a subir devagar | fios longos ou breadboard: baixar `I2C_FREQ_HZ` para 100000 |
| `FALHA_CALIBRACAO` | cubo mexeu nos primeiros 2 s → `R` |
| `FALHA_CICLO_ATRASADO` | outra tarefa a bloquear o Core 1 |
| `yaw` a saltar ±180° | normal: é o envolvimento em [−180°, 180°] |
| Roda a saturar em segundos | esperado se `HAL_RODA_SIMULADA` for 1 em bancada |

## O que falta para fechar a malha

1. **`drv_mcp2518fd.c`** — as três funções declaradas no fim de `drv_moteus.h`:
   `mcp2518fd_iniciar`, `mcp2518fd_enviar`, `mcp2518fd_receber`. O moteus exige
   CAN-FD a 1 Mbit/s na arbitragem e 5 Mbit/s nos dados, ponto de amostragem
   0,666, SJW tão grande quanto possível, e trama **extendida** (o bit 15 do ID
   pede resposta, logo o ID fica acima de 0x7FFF). O protocolo do moteus já
   está feito e verificado em `drv_moteus.c`.
2. **Calibração do mj5208 no moteus**, feita uma vez a partir do PC.
3. **Tarefa da câmara** no Core 0, a chamar
   `app_controlo_publicar_medida_visual()` com o instante da captura tirado do
   mesmo `esp_timer` (exemplo no fim de `app_controlo.c`).

## Fontes

- [Raw CAN-FD — moteus](https://mjbots.github.io/moteus/integration/can-fd/)
- [Register Protocol — moteus](https://mjbots.github.io/moteus/protocol/registers/)
- [CAN Protocol — moteus](https://mjbots.github.io/moteus/protocol/can/)
- [XIAO ESP32-S3 — Seeed Studio](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
