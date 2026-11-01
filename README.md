# Controlo de atitude

## O que este código tem de cumprir e a que necessidades responde

Este módulo implementa a lógica matemática de controlo de atitude prevista
para o projeto do CubeSat suspenso num gimbal. O objetivo é transformar o erro
de apontamento e as velocidades medidas pela IMU em comandos limitados para
as rodas de reação, mantendo o código organizado por eixo e permitindo testar
a resposta antes da integração com o hardware.

### Requisitos funcionais

- **Yaw (guinada):** usar duas malhas em cascata. A malha externa recebe o erro
  angular entre a atitude pretendida e a atitude atual, em radianos, e calcula
  uma velocidade angular desejada. A malha interna compara essa velocidade com
  a velocidade medida pela IMU e calcula o binário a comandar à roda.
- **Saturação e anti-windup:** limitar a velocidade desejada e o binário da roda.
  Limitar também os acumuladores integrais. Quando uma saída está saturada, não
  acumular mais erro no sentido que agravaria a saturação; permitir a integração
  quando o erro ajuda a sair dela.
- **Roll e pitch:** não tentar apontar estes eixos para uma atitude. Aplicar
  amortecimento proporcional à velocidade angular para contrariar as oscilações
  do gimbal, mantendo o binário dentro do limite configurado.
- **Tempo de amostragem:** usar o `dt` recebido em cada chamada para calcular
  integração e derivação. O chamador é responsável por medir e fornecer o tempo
  decorrido desde a amostra anterior.
- **Validação:** ignorar entradas não finitas ou intervalos temporais não
  positivos, em vez de os propagar nos cálculos do controlador.
- **Teste de degrau:** simular uma referência de 45 graus durante 60 ciclos,
  com períodos entre 48 e 52 ms, verificar os comandos de amortecimento e
  saturação, e exigir erro final de no máximo ±1 grau.

### Necessidades a que responde

O controlador de yaw responde à necessidade de orientar o CubeSat para um alvo
e de converter esse erro em comandos que as rodas possam executar. A cascata
separa o comportamento de apontamento (posição) da regulação da velocidade
(atuador). O anti-windup responde aos limites físicos do motor/roda: sem ele,
o integral poderia continuar a crescer enquanto o atuador já não consegue
aumentar o binário. O amortecimento de roll e pitch responde às oscilações
pendulares do gimbal sem introduzir um setpoint de atitude nesses eixos.

Os ganhos, os limites de integral e os limites de saída estão centralizados em
`controlo_atitude.h` para facilitar a leitura e a afinação inicial.

### O que o ensaio demonstra — e o que não demonstra

O programa `teste.c` verifica a execução matemática da cascata e o cumprimento
do critério de erro no **modelo simplificado** usado no próprio teste. Esse
modelo assume uma inércia fixa, não inclui atrito, dinâmica detalhada da roda,
ruído/deriva da IMU, atraso de câmara, perturbações nem quantização do atuador.
O `dt` é uma sequência repetitiva de valores entre 48 e 52 ms, não uma medição
do relógio do sistema.

Assim, o resultado do teste não certifica o desempenho nem a segurança do
hardware. Na aplicação real, será necessário obter a atitude/erro e as
velocidades dos sensores, medir o `dt` no ciclo de controlo, converter o binário
para o formato aceite pelo controlador do motor e validar os limites e ganhos
com o modelo físico e os procedimentos de segurança do laboratório.
