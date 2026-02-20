# Self-Balancing Robot: PID x FLC usando um ATmega328p
Implementação prática de controladores PID e Fuzzy Logic Control em linguagem C para estabilização de um robô autoequilibrante utilizando um ATmega328p.

## Objetivo
Este projeto teve como objetivo implementar e comparar duas técnicas de controle (PID e Fuzzy) em um robô Self-Balancing, analisando desempenho e estabilidade.

## Componentes de Hardware utilizados
O robô foi construído com os seguintes componentes de Hardware:
- Um microcontrolador ATmega328p, utilizando a plataforma Arduino para execução dos algoritmos;
- Um sensor giroscópio/acelerômetro MPU6050, para medição de velocidade angular e aceleração do robô;
- Atuadores (motores DC) para controlar as rodas;
- Um módulo de ponte H L298N, para controle seguro dos motores;
- Alimentação do sistema com duas baterias 18650 (3,7 V cada);
- Uma fonte de protoboard, para converter a tensão de 7,4 V do sistema para 5 V, destinada ao microcontrolador e ao sensor;
- Módulo HC-05 para transferência de dados via sinal Bluetooth diretamente para o terminal serial da IDE.

## Execução do Controle
### PID
O controlador PID calcula o sinal de controle a partir do erro angular, utilizando componentes proporcional, integral e derivativa para compensar inclinação instantânea, erro acumulado e taxa de variação.

### FLC
FLC (Fuzzy logic control) consiste no controle de um sistema por meio de variáveis linguísticas. O importante não é o valor numérico, mas o que ele representa. O controlador recebe os valores de inclinação e velocidade, traduz para variáveis linguísticas, reage com base em regras pré-definidas e devolve uma variável linguística de saída que é convertida para número novamente.

## Arquitetura de Execução
Após a inicialização, o código executa em loop, realizando as seguintes etapas:
1. Os dados de velocidade angular e aceleração do MPU6050 são coletados através de comunicação I2C.
2. O ângulo de inclinação do robô é obtido através desses dados.
3. O valor do ângulo, comparado com a inclinação neutra, é enviado como entrada do algoritmo de controle (PID ou FLC).
4. O algoritmo de controle retorna um valor que é interpretado, pelo comando de acionamento dos motores, como um sinal PWM.

## Temporização do Controle
A execução do algoritmo ocorre em loop contínuo, sem uso de delay para temporização fixa do controle. A taxa de atualização do sistema é determinada pelo tempo de execução do ciclo completo de leitura do sensor, execução do controle e atualização do sinal PWM.
A função `millis()` é utilizada no algoritmo de envio opcional de dados via comunicação Bluetooth para um terminal serial. A comunicação pode gerar uma pequena variação no período de execução do loop, alterando levemente a dinâmica do sistema. Nesse caso, é necessário um ajuste fino na calibragem das constantes de PID e na base de regras do controle Fuzzy.

## Estrutura do Código
- `Algoritmos de Controle/self_balancing_PID/self_balancing_PID.ino` - Implementação do controle PID
- `Algoritmos de Controle/self_balancing_FLC/self_balancing_FLC.ino` - Implementação do controle Fuzzy

## Como Usar
1. Abra o arquivo desejado.
2. Faça o download do código ou copie-o para o seu arquivo.
3. Carregue o código no seu microcontrolador.
Observação: os arquivos estão em formato .ino. Ambos foram projetados originalmente para o microcontrolador ATmega328p, utilizando a Arduino IDE. ESP e outros microcontroladores podem exigir adaptação de código, mesmo que seus códigos ainda possam ser desenvolvidos na Arduino IDE. Veja o tópico "Adaptação para Outras Plataformas" para adaptar o código para ser executado em plataformas sem quaisquer relações com a plataforma Arduino.

## Adaptação para Outras Plataformas
Caso o microcontrolador utilizado não seja da família ATmega, ou não suporte programação através da IDE do Arduino, mas ainda possa ser programado na linguagem C, alguns pontos dos códigos devem ser adaptados. Ambos os códigos estão separados por funções, algumas das quais podem ser copiadas integralmente, desde que o microcontrolador utilizado seja compatível com a linguagem C. Todas as variáveis globais e constantes podem ser copiadas integralmente. Algumas bibliotecas utilizadas podem gerar incompatibilidade, dependendo do microcontrolador utilizado, caso tenham sido desenvolvidas exclusivamente para Arduino.

### Funções do MPU6050
O MPU6050 se comunica com o microcontrolador através do protocolo I2C, sendo necessárias uma linha de sincronização e outra de transferência de dados. A leitura, feita através da biblioteca `Wire.h`, consiste em requisitar os dados do acelerômetro e do giroscópio do sensor, seguida pela conversão de valores brutos para grandezas físicas.
- `mpu_config` - Configura o MPU6050 com base em uma calibragem realizada previamente.
- `setOffset` - Função chamada pela calibragem do sensor, para calibrar cada eixo individualmente.
- `mpu_init` - Função de inicialização do sensor.
- `mpu_data` - Função de coleta de dados de velocidade angular e aceleração do sensor MPU6050.

### Função do módulo Bluetooth
- `hc_init` - Pode ser inteiramente apagada, em caso de não utilização do módulo Bluetooth.

### Funções que necessitam de alterações
Dependendo do microcontrolador utilizado, essas funções devem ser alteradas.
- `motor_init` - Inicia os pinos dos motores. Os comandos `pinMode` devem ser substituídos pelo comando de inicialização de pinos do microcontrolador utilizado. Todos os pinos utilizados nessa função devem ser definidos como saídas (Output).

### Funções que podem ser copiadas integralmente
Essas funções podem ser copiadas, desde que o microcontrolador utilizado seja programado em linguagem C.
- `pid_control` - Realiza os cálculos do controle PID (no caso do arquivo `self_balancing_PID.ino`).
- `motor_control` - Realiza o controle dos motores através do valor da variável `mspeed` (no caso do arquivo `self_balancing_PID.ino`).
- `fuzzy_init` - Realiza a definição da base de regras para o controle Fuzzy (no caso do arquivo `self_balancing_FLC.ino`).
- `flc_out` - Realiza o controle do sistema pela técnica Fuzzy (no caso do arquivo `self_balancing_FLC.ino`).
- `set_motor` - Realiza o controle dos motores através do valor da variável `pwm` (no caso do arquivo `self_balancing_FLC.ino`).
Observação: o controle dos motores é feito através da geração de um sinal PWM, pelo comando `analogWrite`, responsável por configurar a largura do pulso do sinal enviado à ponte-H, com base no valor obtido na saída do controlador. Em outras plataformas não relacionadas ao Arduino, é necessário adaptar o comando. Já na utilização de registradores do microcontrolador, mesmo no ATmega328p, o sinal deve ser configurado através do timer correspondente ao pino utilizado.

### Funções `setup()` e `loop()`
A função `setup` pode ser copiada dentro da função `main` do seu código. A função `loop`, por sua vez, pode ser chamada dentro da função `main`, mas sua chamada deve estar dentro de um laço de repetição infinito (como `while(1)`).

## Resultados do Projeto
Ambos os controladores conseguiram manter o robô em equilíbrio por aproximadamente 30 segundos, sob as mesmas condições físicas.
O controle PID mostrou maiores oscilações perceptíveis e variação do sinal PWM. O controle Fuzzy, por sua vez, manteve o robô com menos oscilações perceptíveis, com sinal PWM mais baixo e amplitude de oscilação menor.
Sob as condições testadas, o controle Fuzzy demonstrou maior estabilidade e suavidade no equilíbrio.
