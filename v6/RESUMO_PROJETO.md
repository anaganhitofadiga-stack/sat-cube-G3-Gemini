# Controlo de Atitude de um Cubo via Rodas de Reação e Visão Computacional

## Resumo do Trabalho Realizado

### Arquitetura de Hardware & Software
- Sistema embutido baseado em microcontrolador ESP32-S3 dual-core com FreeRTOS
- Core 1 dedicado ao controlo de atitude crítico (IMU, malha de controlo, motores)
- Core 2 para aquisição de imagem e comunicação com a Ground Station
- Sensores: MPU-6050 (IMU), LIS3MDL (magnetómetro), OV5640 (câmara)
- Atuadores: 3 motores de reação (moteus r4.11) com comunicação CAN-FD via MCP2518FD
- Alimentação por bateria com monitorização de tensão e buzzer de alerta

### Firmware Desenvolvido (pasta `firmware/`)
- Arquitetura multicore com prioridades de tempo-real (pasta `main/`)
- Modularidade via interfaces de hardware (HAL) e device drivers (pasta `main/`)
- Aplicação de controlo (`app_controlo.c`) com máquina de estados e modos de operação 
- Malha de controlo em cascata PID para atitude (pasta `controlo_atitude/`)
- Processamento de imagem para tracking visual de alvo (pasta `controlo_visual/`)
- Camada de telecomandos e telemetria por porta série USB-JTAG (`app_usb.[ch]`)
- Makefile e configurações para o ESP-IDF (`CMakeLists.txt`, `sdkconfig.defaults`)

### Ground Station & Ferramentas
- Interface gráfica em Python com PyQt5 para visualização e controlo (pasta `ferramentas/`)
- Registo de telemetria em ficheiro CSV e plot em tempo-real dos sinais
- Injeção de telecomandos por terminal com parsing robusto
- Simulação "hardware-in-the-loop" (HIL) do firmware antes da integração física

### Documentação
- Descrição detalhada da arquitetura de controlo e fundamentos teóricos (pasta `docs/`)
- Protocolo de comunicação (ICD) com definição das tramas e validação CRC
- Procedimentos de teste e validação seguindo o modelo em V (pasta `testes/`)
- Registos de bugs, melhorias e alterações em CHANGELOG.md
- Guias de compilação, programação e utilização no README.md

## Estado Atual do Projeto

### Funcionalidades Implementadas
1. Aquisição de dados de atitude da IMU (taxa de 200Hz)
2. Filtragem do giroscópio e acelerómetro com filtro complementar 
3. Estimativa de atitude 3D (roll, pitch, yaw) com desacoplamento dos eixos
4. Controlador em cascata PID (posição ⟶ velocidade ⟶ binário) para o eixo Yaw
5. Protocolo CAN-FD para envio de binários aos motores e leitura de velocidade
6. Telemetria em tempo-real via porta USB com taxa de amostragem ajustável
7. Visualização de atitude, velocidades, binários e sinais de controlo na Ground Station
8. Telecomandos para seleção de modo (Rate, Attitude, Visual), referências e ganhos
9. Aquisição de imagem monocromática da câmara OV5640 com compressão JPEG
10. Deteção do centroide de um alvo visual por binarização e momentos de imagem
11. Ajuste do yaw com base no desvio horizontal do alvo para o centro da imagem
12. Modos de segurança: desarmamento remoto, corte de binário por timeout e limites de velocidade

### Tarefas em Aberto
1. Afinação dos ganhos dos controladores em cascata para resposta ótima
2. Rejeição de perturbações no yaw devido a acoplamento giroscópico dos motores
3. Estimativa de distância do alvo visual por tamanho aparente ou focagem 
4. Integração do magnetómetro para correção da deriva em yaw (modo Attitude)
5. Otimizações no pipeline de processamento de imagem (DMA e double-buffering)
6. Adição de watchdogs e deteção de falhas nos periféricos (SPI, I2C, CAN)
7. Implementação de um canal de telemetria redundante por Bluetooth Low-Energy
8. Expansão dos testes unitários em Catch2 para aumentar a cobertura do código
9. Ensaios de autonomia da bateria sob perfis de uso representativos

## Contribuições dos Membros da Equipa

- **Ana:** Modelação da dinâmica e simulação (MATLAB/Simulink), estimativa de atitude (filtragem complementar, desacoplamento), afinação dos ganhos PID para amortecimento em roll/pitch e controlo do yaw.

- **Maurício:** Arquitetura de controlo em cascata (atitude ⟶ vel. angular ⟶ vel. roda), definição da hierarquia dos device drivers e camada HAL, revisão do código para otimização e legibilidade.

- **Ricardo:** Protocolo de comunicação (definição do formato das tramas, numeração dos registos, endereçamento), implementação da validação CRC e retransmissão seletiva, elaboração do ICD.

- **Edson:** Desenvolvimento do firmware com foco no FreeRTOS (gestão de tarefas, sincronização, filas, partições), device drivers para periféricos (I2C, SPI, CAN), otimização para desempenho em tempo-real.

- **Alessia:** Implementação da Ground Station (GUI em PyQt5 com gráficos de telemetria em tempo-real), registo em CSV, parseamento dos telecomandos, comunicação com a porta série.

Em caso de dúvidas, consultar a documentação detalhada na pasta `docs/` ou o repositório GIT do projeto. O próximo marco importante é a conclusão da validação cruzada nível 3 com ensaios da resposta ao degrau, a realizar até dia 19 de Novembro. 