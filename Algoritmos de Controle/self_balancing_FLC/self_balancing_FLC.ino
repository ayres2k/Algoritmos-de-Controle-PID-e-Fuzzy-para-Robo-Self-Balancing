/*****************************************************************\

  Código utilizado no Trabalho de Conclusão de Curso
  Medidas de Desempenho e Comparação PID x FLC Usando um
  Self-Balancing Robot

  Código adaptado por Felipe Vieira Ayres de Oliveira

  Código original encontrado em:
  https://github.com/KoKoLates/eFLL-balancing-robot

  Funções de Pertinência baseadas no código do link:
  https://github.com/ahmaadfaauzn2/Balancia

\*****************************************************************/

//================================================================================================================================================================================//
// Bibliotecas
# include <Fuzzy.h> //---------------------------------------------------------------Biblioteca EFLL (Embedded Fuzzy Logic Library): https://github.com/alvesoaj/eFLL
# include <Wire.h> //----------------------------------------------------------------Biblioteca de comunicação i2C, necessário para o funcionamento do MPU-6050

// Identificação de pinos
# define rmotor1 11
# define rmotor2 6
# define lmotor1 5
# define lmotor2 3

//================================================================================================================================================================================//
// Variáveis
Fuzzy *fuzzy = new Fuzzy(); //-------------------------------------------------------Objeto principal do sistema fuzzy

//================================================================================================================================================================================//
// Variáveis
int16_t acc_rawX, acc_rawY, acc_rawZ; //---------------------------------------------Variáveis brutas do acelerômetro
int16_t gyr_rawX, gyr_rawY, gyr_rawZ; //---------------------------------------------Variáveis brutas do giroscópio

volatile float accAngle; //----------------------------------------------------------Ângulo do acelerômetro
volatile float gyroRate; //----------------------------------------------------------Velocidade do giroscópio
volatile float Angle; //-------------------------------------------------------------Valor final do ângulo
volatile float flc; //---------------------------------------------------------------Saída do controlador
float offset = 0.0; //---------------------------------------------------------------Offset, se necessário

unsigned long t0 = 0, t1 = 0; //-----------------------------------------------------Controle do tempo entre leituras
unsigned long t_init = 0; //---------------------------------------------------------Marcação do início do primeiro loop
float t_amostra = 0; //--------------------------------------------------------------Marcação do período até a amostragem
float periodo = 0; //----------------------------------------------------------------Período das leituras

float rad_to_deg = 180.0 / 3.141592654;

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
  Wire.write(0x00);
  Wire.endTransmission(true);
  mpu_config(); //-------------------------------------------------------------------Chamada da função de calibragem do sensor MPU6050
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
  // Inicia coleta de dados do sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  // Coleta de dados brutos do acelerômetro
  acc_rawX = Wire.read() <<8 | Wire.read();
  acc_rawY = Wire.read() <<8 | Wire.read();
  acc_rawZ = Wire.read() <<8 | Wire.read();

  Wire.read(); Wire.read(); //-------------------------------------------------------Temperatura lida, mas ignorada e não armazenada

  // Coleta de dados brutos do giroscópio
  gyr_rawX = Wire.read() <<8 | Wire.read();
  gyr_rawY = Wire.read() <<8 | Wire.read();
  gyr_rawZ = Wire.read() <<8 | Wire.read();

  // Conversão de dados brutos para ângulo do acelerômetro e velocidade do giroscópio
  accAngle = atan2(acc_rawY, acc_rawZ) * rad_to_deg;
  gyroRate = gyr_rawX/131.0;
}

//================================================================================================================================================================================//
// Construção do controlador fuzzy: funções de pertinência e base de regras
void fuzzy_init()
{
  // Observação: pode ser necessário configurar as funções de pertinência a depender da estrutura do robô

  // Conjuntos Fuzzy para o Ângulo
  FuzzySet *angle_n = new FuzzySet(-10,  -10,  -5,   0); //--------------------------Ângulos negativos
  FuzzySet *angle_z = new FuzzySet(-3,     0,   0,   3); //--------------------------Ângulos próximos de zero
  FuzzySet *angle_p = new FuzzySet( 0,     5,  10,  10); //--------------------------Ângulos positivos

  // Conjuntos Fuzzy para a Velocidade Angular
  FuzzySet *vel_n =  new FuzzySet(-40,  -40,  -10,   0); //--------------------------Velocidades negativas
  FuzzySet *vel_z =  new FuzzySet( -8,    0,    0,   8); //--------------------------Velocidades próximas de 0
  FuzzySet *vel_p =  new FuzzySet(  0,   10,   40,  40); //--------------------------Velocidades positivas

  // Conjuntos Fuzzy para a saída do motor (PWM)
  FuzzySet *power_min  = new FuzzySet( 10,  10,  10,  10); //------------------------Potência mínima
  FuzzySet *power_low  = new FuzzySet( 20,  20,  20,  20); //------------------------Potência baixa
  FuzzySet *power_med  = new FuzzySet( 50,  50,  50,  50); //------------------------Potência média
  FuzzySet *power_high = new FuzzySet(150, 150, 150, 150); //------------------------Potência alta
  FuzzySet *power_max  = new FuzzySet(255, 255, 255, 255); //------------------------Potência máxima

  // Base de Regras
  /*
      Tabela de saídas FLC com base em Ângulo x Velocidades
      |Ang\Vel|--N---|--Z---|--P---|
      |--N----|-MAX--|-HIGH-|-MED--|
      |--Z----|-LOW--|-MIN--|-LOW--|
      |--P----|-MED--|-HIGH-|-MAX--|
  */

  // Atribuição das Entradas e Saídas às suas respectivas funções de pertinência
  FuzzyInput *angle_input = new FuzzyInput(1); //------------------------------------Ângulos como Entrada 1
  angle_input->addFuzzySet(angle_n);
  angle_input->addFuzzySet(angle_z);
  angle_input->addFuzzySet(angle_p);
  fuzzy->addFuzzyInput(angle_input);

  FuzzyInput *vel_input = new FuzzyInput(2); //--------------------------------------Velocidades como Entrada 2
  vel_input->addFuzzySet(vel_n);
  vel_input->addFuzzySet(vel_z);
  vel_input->addFuzzySet(vel_p);
  fuzzy->addFuzzyInput(vel_input);

  FuzzyOutput *power_out = new FuzzyOutput(1); //------------------------------------Potências (PWM) como saída
  power_out->addFuzzySet(power_min);
  power_out->addFuzzySet(power_low);
  power_out->addFuzzySet(power_med);
  power_out->addFuzzySet(power_high);
  power_out->addFuzzySet(power_max);
  fuzzy->addFuzzyOutput(power_out);

  // angle_Z & vel_Z -> Potência Mínima
  FuzzyRuleAntecedent *zz = new FuzzyRuleAntecedent(); //----------------------------Ângulo e velocidade próximos de 0
  FuzzyRuleConsequent *output_min = new FuzzyRuleConsequent(); //--------------------Saída mínima
  zz->joinWithAND(angle_z, vel_z); //------------------------------------------------União de Conjuntos Fuzzy
  output_min->addOutput(power_min); //-----------------------------------------------Obtenção da Saída
  fuzzy->addFuzzyRule(new FuzzyRule(1, zz, output_min)); //--------------------------Regra 1 criada com entrada zz e saída output_min


  // (angle_Z & vel_N) || (angle_Z & vel_P) -> Potência baixa
  FuzzyRuleAntecedent *zn = new FuzzyRuleAntecedent(); //----------------------------Ângulo próximo de 0 e velocidade negativa
  FuzzyRuleAntecedent *zp = new FuzzyRuleAntecedent(); //----------------------------Ângulo próximo de 0 e velocidade positiva
  FuzzyRuleAntecedent *cond_low = new FuzzyRuleAntecedent(); //----------------------Variável para unir (OU) duas sub-regras em uma regra maior
  FuzzyRuleConsequent *output_low = new FuzzyRuleConsequent(); //--------------------Saída baixa
  zn->joinWithAND(angle_z, vel_n); //------------------------------------------------União de Conjuntos Fuzzy
  zp->joinWithAND(angle_z, vel_p);
  cond_low->joinWithOR(zn, zp); //---------------------------------------------------Interseção de Conjuntos Fuzzy
  output_low->addOutput(power_low); //-----------------------------------------------Obteção da Saída
  fuzzy->addFuzzyRule(new FuzzyRule(2, cond_low, output_low)); //--------------------Regra 2 criada com entradas zn OU zp e saída output_low


  // (angle_N & vel_P) || (angle_P & vel_N) -> Potência média
  FuzzyRuleAntecedent *np = new FuzzyRuleAntecedent(); //----------------------------Ângulo negativo e velocidade positiva
  FuzzyRuleAntecedent *pn = new FuzzyRuleAntecedent(); //----------------------------Ângulo positivo e velocidade negativa
  FuzzyRuleAntecedent *cond_med = new FuzzyRuleAntecedent(); //----------------------Variável para unir (OU) duas sub-regras em uma regra maior
  FuzzyRuleConsequent *output_med = new FuzzyRuleConsequent(); //--------------------Saída média
  np->joinWithAND(angle_n, vel_p); //------------------------------------------------União de Conjuntos Fuzzy
  pn->joinWithAND(angle_p, vel_n);
  cond_med->joinWithOR(np, pn); //---------------------------------------------------Interseção de Conjuntos Fuzzy
  output_med->addOutput(power_med); //-----------------------------------------------Obtenção da Saída
  fuzzy->addFuzzyRule(new FuzzyRule(3, cond_med, output_med)); //--------------------Regra 3 criada com entradas np OU pn e saída output_med


  // (angle_N & vel_Z) || (angle_P & vel_Z) -> Potência alta
  FuzzyRuleAntecedent *nz = new FuzzyRuleAntecedent(); //----------------------------Ângulo negativo e velocidade próxima de zero
  FuzzyRuleAntecedent *pz = new FuzzyRuleAntecedent(); //----------------------------Ângulo positivo e velocidade próxima de zero
  FuzzyRuleAntecedent *cond_high = new FuzzyRuleAntecedent(); //---------------------Variável para unir (OU) duas sub-regras em uma regra maior
  FuzzyRuleConsequent *output_high = new FuzzyRuleConsequent(); //-------------------Saída alta
  nz->joinWithAND(angle_n, vel_z); //------------------------------------------------União de Conjuntos Fuzzy
  pz->joinWithAND(angle_p, vel_z);
  cond_high->joinWithOR(nz, pz); //--------------------------------------------------Interseção de Conjuntos Fuzzy
  output_high->addOutput(power_high); //---------------------------------------------Obtenção da Saída
  fuzzy->addFuzzyRule(new FuzzyRule(4, cond_high, output_high)); //------------------Regra 4 criada com entradas nz OU pz e saída output_high


  // (angle_N & vel_N) || (angle_P & vel_P) -> Potência máxima
  FuzzyRuleAntecedent *nn = new FuzzyRuleAntecedent(); //----------------------------Ângulo e velocidade negativos
  FuzzyRuleAntecedent *pp = new FuzzyRuleAntecedent(); //----------------------------Ângulo e velocidade positivos
  FuzzyRuleAntecedent *cond_max = new FuzzyRuleAntecedent(); //----------------------Variável para unir (OU) duas sub-regras em uma regra maior
  FuzzyRuleConsequent *output_max = new FuzzyRuleConsequent(); //--------------------Saída máxima
  nn->joinWithAND(angle_n, vel_n); //------------------------------------------------União de Conjuntos Fuzzy
  pp->joinWithAND(angle_p, vel_p);
  cond_max->joinWithOR(nn, pp); //---------------------------------------------------Interseção de Conjuntos Fuzzy
  output_max->addOutput(power_max); //-----------------------------------------------Obtenção da Saída
  fuzzy->addFuzzyRule(new FuzzyRule(5, cond_max, output_max)); //--------------------Regra 5 criada com entradas nn OU pp e saída output_max
}

//================================================================================================================================================================================//
// Controlador Fuzzy
void flc_out()
{
  // Conversão de dados do acelerômetro
  Angle = 0.98 *(Angle + gyroRate * periodo) + 0.02 * accAngle;
  
  // Controlador Fuzzy
  fuzzy -> setInput(1, Angle-offset); //---------------------------------------------Entrada 1: ângulo
  fuzzy -> setInput(2, gyroRate); //-------------------------------------------------Entrada 2: Velocidade
  fuzzy -> fuzzify(); //-------------------------------------------------------------Fuzzifica as entradas
  flc = fuzzy -> defuzzify(1); //----------------------------------------------------Defuzzifica a saída e atribui à variável flc
}

//================================================================================================================================================================================//
// Controle dos motores
void setMotor(int pwm)
{
  // Observação: Offset sempre é considerado, mas, por padrão, é igual a 0. Deve ser alterado na declaração da variável, caso necessário
  if(Angle-offset >= 0)
  {
    analogWrite(rmotor1, pwm);
    analogWrite(rmotor2, 0);
    analogWrite(lmotor1, pwm);
    analogWrite(lmotor2, 0);
  }
  else if(Angle-offset < 0)
  {
    analogWrite(rmotor1, 0);
    analogWrite(rmotor2, pwm);
    analogWrite(lmotor1, 0);
    analogWrite(lmotor2, pwm);
  }
}

//================================================================================================================================================================================//
// Função principal (execução única)
void setup()
{
  // Inicialização dos componentes
  hc_init(); //----------------------------------------------------------------------HC-06, para comunicação Bluetooth com o monitor serial do Arduino IDE
  mpu_init(); //---------------------------------------------------------------------Inicialização do MPU6050
  motor_init(); //-------------------------------------------------------------------Inicialização dos motores
  setMotor(0); //--------------------------------------------------------------------Configura a saída pwm inicial para 0
  fuzzy_init(); //-------------------------------------------------------------------Construção da base de regras de fuzzy
  t0 = millis(); //------------------------------------------------------------------Marca o tempo inicial do primeiro loop, para cálculo de período do loop
  t_init = millis(); //--------------------------------------------------------------Marca o tempo inicial do primeiro loop, para cálculo do tempo total de execução dos loops
}

//================================================================================================================================================================================//
// Função principal (execução em loop)
void loop()
{
  // Marcações de tempos inicias e finais, e período
  t0 = t1;
  t1 = millis();
  periodo = (t1 - t0) / 1000.0;
  t_amostra += periodo;

  mpu_data(); //---------------------------------------------------------------------Obtenção dos dados do MPU-6050
  flc_out(); //----------------------------------------------------------------------Etapa de controle fuzzy
  int pwm = constrain(flc, 0, 255); //-----------------------------------------------Garante que os valores de PWM estarão entre 0 e 255

  if(pwm >= 25 && pwm <= 45)  //-----------------------------------------------------Garante que os motores irão iniciar/parar de forma sincronizada
  {
    pwm = 45;
    /*
      Observação: deve-se verificar o início da rotação de cada motor em ambas as direções, antes de definir esses valores.
      Recomendação: o intervalo deve ser entre o valor de PWM que um dos motores começa a girar em uma direção específica,
      e o valor de PWM em que os motores giram em ambas as direções. O valor final de pwm precisa ser maior ou igual ao
      limite superior desse intervalo
    */
  }
  setMotor(pwm); //------------------------------------------------------------------Os motores giram com base no valor de PWM

  // Escrita dos dados de tempo total de execução, inclinação do robô e valor PWM no monitor serial, a cada 50ms.
  /*
    Observação: caso não utilizar essas leituras, comente todo esse trecho de código. Pequenos ajustes nas funções
    de pertinência podem ser necessários para manter o robô em equilíbrio.
  */
  if(t_amostra >= 0.05)
  {
    Serial.print(millis()-t_init);    Serial.print(",");
    Serial.print(Angle-offset); Serial.print(",");
    Serial.println(pwm);
    t_amostra = 0;
  }
}
/*****************************************************************\

  Código utilizado no Trabalho de Conclusão de Curso
  Medidas de Desempenho e Comparação PID x FLC Usando um
  Self-Balancing Robot

  Código adaptado por Felipe Vieira Ayres de Oliveira

  Código original encontrado em:
  https://github.com/KoKoLates/eFLL-balancing-robot

  Funções de Pertinência baseadas no código do link:
  https://github.com/ahmaadfaauzn2/Balancia

\*****************************************************************/