/*****************************************************************\

  Código utilizado no Trabalho de Conclusão de Curso
  Medidas de Desempenho e Comparação PID x FLC Usando um
  Self-Balancing Robot

  Código adaptado por Felipe Vieira Ayres de Oliveira

  Código original encontrado em:
  https://www.instructables.com/Self-Balancing-Bot-Using-PID-Control-System/

\*****************************************************************/

//================================================================================================================================================================================//
// Bibliotecas

#include <Wire.h> //-----------------------------------------------------------------Comunicação i2C com MPU6050

//================================================================================================================================================================================//
// Identificação de pinos
#define rmotor1 11
#define rmotor2 6
#define lmotor1 5
#define lmotor2 3
int mspeed = 0; //-------------------------------------------------------------------Controle de velocidade dos motores

//================================================================================================================================================================================//
// Variáveis
int16_t Acc_rawX, Acc_rawY, Acc_rawZ; //---------------------------------------------Variáveis brutas do Acelerômetro
int16_t Gyr_rawX, Gyr_rawY, Gyr_rawZ; //---------------------------------------------Variáveis brutas do Giroscópio
float Acceleration_angle[2]; //------------------------------------------------------Ângulos do Acelerômetro
float Gyro_angle[2]; //--------------------------------------------------------------Ângulos do Giroscópio
float Total_angle[2]; //-------------------------------------------------------------Valor final de ângulos
float rad_to_deg = 180/3.141592654;

float elapsedTime, time, timePrev, t0; //--------------------------------------------Variáveis de tempo
float PID, error = 0, previous_error = 0; //-----------------------------------------Variáveis de entrada e saída do controlador
float t_amostra = 0;

float desired_angle = 0.28; //-------------------------------------------------------Ângulo desejado, com pequeno offset para corrigir problema dos motores

float pid_P; //----------------------------------------------------------------------Componente proporcional do controlador
float pid_I; //----------------------------------------------------------------------Componente integral do controlador
float pid_D; //----------------------------------------------------------------------Componente derivativa do controlador

//================================================================================================================================================================================//
// Ganhos do controlador PID
// Observação: valores de kp, ki e kd devem ser configurado com base no seu robô.
float kp = 57; //--------------------------------------------------------------------Saída proporcional ao erro de inclinação do robô
float ki = 0.8; //-------------------------------------------------------------------Integra o erro pelo tempo para corrigir pequenos acúmulos ao longo do tempo
float kd = 0.4; //-------------------------------------------------------------------Deriva o erro pelo tempo para determinar sentido e intensidade da inclinação


//================================================================================================================================================================================//
// Calibração do sensor MPU6050
void mpu_config()  //----------------------------------------------------------------Calibração do sensor MPU6050
{
  /*
    Observação: um código externo foi usado para calibrar o acelerômetro e o giroscópio,
    e pode ser encontrado como exemplo nos arquivos da biblioteca MPU6050. O código foi
    executado com o sensor estático em uma superfície plana, e retornou os valores de
    correção de offset utilizados abaixo. Os valores abaixo são referentes ao sensor
    utilizado neste projeto, e podem variar entre diferentes sensores.
  */

  // Calibração do Acelerômetro (byte_correspondente_ao_dado, valor_de_calibracao)
  setOffset(0x06, -3604); //---------------------------------------------------------Chamadas da função que executa a calibração de cada eixo do acelerômetro e do giroscópio
  setOffset(0x08, 2636);
  setOffset(0x0A, 2896);

  // Calibração do Giroscópio (byte_correspondente_ao_dado, valor_de_calibracao)
  setOffset(0x13, 121);
  setOffset(0x15, -97);
  setOffset(0x17, 297);
}

void setOffset(uint8_t regHigh, int16_t valor)  //-----------------------------------Correção de offsets do sensor MPU6050
{
  uint8_t hByte = (valor >> 8) & 0xFF;
  uint8_t lByte = valor & 0xFF;

  Wire.beginTransmission(0x68);
  Wire.write(regHigh);
  Wire.write(hByte);
  Wire.write(lByte);
  Wire.endTransmission();
}

//================================================================================================================================================================================//
// Configuração inicial do MPU6050
void mpu_init()
{
  // Inicia o MPU6050
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Chamada da função de calibragem do MPU6050
  mpu_config();
}

//================================================================================================================================================================================//
// Configuração inicial dos pinos dos motores
void motor_init()
{
  pinMode(rmotor1, OUTPUT);
  pinMode(rmotor2, OUTPUT);
  pinMode(lmotor1, OUTPUT);
  pinMode(lmotor2, OUTPUT);
}

//================================================================================================================================================================================//
// Configuração inicial da comunicação bluetooth via HC-05
void hc_init()
{
  Serial.begin(9600);
  Serial.println("\n\nInicio\n\n");
}

//================================================================================================================================================================================//
// Comunicação com o MPU6050
void mpu_data()
{
  // Coleta de dados brutos do acelerômetro
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); //--------------------------------------------------Coleta de 6 bytes de informação, sendo 2 de cada eixo do acelerômetro
  Acc_rawX = Wire.read() <<8 | Wire.read();
  Acc_rawY = Wire.read() <<8 | Wire.read();
  Acc_rawZ = Wire.read() <<8 | Wire.read();

  // Coleta de dados brutos do giroscópio
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //--------------------------------------------------Coleta de 4 bytes de informação, sendo 2 para cada eixo (excluindo Z) do giroscópio
  Gyr_rawX = Wire.read() <<8 | Wire.read();
  Gyr_rawY = Wire.read() <<8 | Wire.read();

  // Conversão de dados do acelerômetro
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

  // Conversão de dados do giroscópio
  Gyro_angle[0] = Gyr_rawX/131.0;
  Gyro_angle[1] = Gyr_rawY/131.0;

  // Combinação de ambos os dados através do filtro complementar, obtendo os ângulos finais nos eixos X e Y
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0]; // Ângulo final no Eixo X
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1]; // Ângulo final no Eixo Y
}

//================================================================================================================================================================================//
// Controlador PID
void pid_control()
{
  error = Total_angle[0] - desired_angle; //-----------------------------------------Definição do erro como inclinação atual - setpoint
  pid_P = kp*error; //---------------------------------------------------------------Cálculo da componente P
  pid_I = pid_I + (ki*error); //-----------------------------------------------------Cálculo da componente I
  pid_D = kd * ((error - previous_error)/elapsedTime); //----------------------------Cálculo da componente D
  PID = pid_P + pid_I + pid_D; //----------------------------------------------------Cálculo da saída de PID
}

//================================================================================================================================================================================//
// Controle dos motores
void motor_control()
{
  //Observação: Ângulos muito grandes, com magnitude acima de 45º, desligam as rodas.
  if(Total_angle[0]<0)
  {
    analogWrite(rmotor1, 0);
    analogWrite(rmotor2, mspeed);
    analogWrite(lmotor1, 0);
    analogWrite(lmotor2, mspeed);
  }
  else if(Total_angle[0]>0)
  {
    analogWrite(rmotor1, mspeed);
    analogWrite(rmotor2, 0);
    analogWrite(lmotor1, mspeed);
    analogWrite(lmotor2, 0);
  }
  else if(Total_angle[0]>45 || Total_angle[0] <-45)
  {
    analogWrite(rmotor1, 0);
    analogWrite(rmotor2, 0);
    analogWrite(lmotor1, 0);
    analogWrite(lmotor2, 0);
  }
}

//================================================================================================================================================================================//
// Função principal (execução única)
void setup()
{
  hc_init(); //----------------------------------------------------------------------HC-06, para comunicação Bluetooth com o monitor serial do Arduino IDE
  mpu_init(); //---------------------------------------------------------------------Inicialização do MPU6050
  motor_init(); //-------------------------------------------------------------------Inicialização dos motores
  mspeed = 0; //---------------------------------------------------------------------Configura a saída pwm inicial para 0
  motor_control(); //----------------------------------------------------------------Com mspeed=0, os motores iniciam em repouso
  time = millis(); //----------------------------------------------------------------Marca o tempo inicial do primeiro loop, para cálculo de período do loop
  t0 = time; //----------------------------------------------------------------------Marca o tempo inicial do primeiro loop, para cálculo do tempo total de execução dos loops
}

//================================================================================================================================================================================//
// Função principal (execução em loop)
void loop()
{
  // Marcações de tempos iniciais e finais, e período
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;
  t_amostra += elapsedTime;

  mpu_data(); //---------------------------------------------------------------------Obtenção dos dados do MPU-6050
  pid_control(); //------------------------------------------------------------------Etapa de controle PID
  previous_error = error; //---------------------------------------------------------Armazena o valor do erro anterior, para cálculos futuros

  mspeed = constrain(abs(PID), 0, 255); //-------------------------------------------Garante que o valor de PWM esteja entre 0 e 255, considerando valores originalmente negativos

  if(mspeed >= 25 && mspeed <= 45)  //-----------------------------------------------Garante que os motores irão iniciar/parar de forma sincronizada
  {
    mspeed = 45;
    /*
      Observação: deve-se verificar o início da rotação de cada motor em ambas as direções, antes de definir esses valores.
      Recomendação: o intervalo deve ser entre o valor de PWM que um dos motores começa a girar em uma direção específica,
      e o valor de PWM em que os motores giram em ambas as direções. O valor final de pwm precisa ser maior ou igual ao
      limite superior desse intervalo
    */
  }

  motor_control();  //---------------------------------------------------------------Os motores giram com base no valor de PWM

  // Escrita dos dados de tempo total de execução, inclinação do robô e valor PWM no monitor serial, a cada 50ms.
  /*
    Observação: caso não utilizar essas leituras, comente todo esse trecho de código. Pequenos ajustes nas funções
    de pertinência podem ser necessários para manter o robô em equilíbrio.
  */
  if(t_amostra >= 0.05)
  {
    Serial.print(millis()-t0);    Serial.print(",");
    Serial.print(Total_angle[0]); Serial.print(",");
    Serial.println(mspeed);
    t_amostra = 0;
  }
}

/*****************************************************************\

  Código utilizado no Trabalho de Conclusão de Curso
  Medidas de Desempenho e Comparação PID x FLC Usando um
  Self-Balancing Robot

  Código adaptado por Felipe Vieira Ayres de Oliveira

  Código original encontrado em:
  https://www.instructables.com/Self-Balancing-Bot-Using-PID-Control-System/

\*****************************************************************/