/*
 * Comunicação Gsap com o gateway para fazer a requisição dos parâmetros  referente aos links da rede
 * Comunicação modbus com o node red para envio dos dados para monitoramento
 */
#include <WiFi.h> // Instancia um objeto WiFiClient (canalWiFiGSAP) que será enviado como parâmetro para instancia do objeto da GSAP.h
#include <GSAP.h> // Instancia um objeto Gsap (canalGSAP)
#include <PID_v1.h>
#include <ModbusIP_ESP8266.h> // Instancia um objeto ModbusIP(modbus_nodeRed) para o envio dos dados obtidos do gtwy
#include <analogWrite.h>
#include <MODBUS_ESP32.h> // Instancia os objetos para a requisição dos valores de PV ao Gateway via modbus


// Configuração WiFi
#ifndef STASSID
#define STASSID "LampLABA"          //login
#define STAPSK  "udtqcSSONDodtqq"   //senha
#endif
const char* ssid = STASSID;
const char* password = STAPSK;
IPAddress ip(10, 13, 103, 52);
IPAddress gateway(10, 13, 96, 1);
IPAddress subnet(255, 255, 224, 0);
char* hostGW = "10.13.103.42";
uint16_t netID = 100;

// Objetos instaciados
WiFiClient canalWiFiGSAP;
WiFiClient canalMODBUS_GW;
Gsap canalGSAP(&canalWiFiGSAP,netID,hostGW);
ModbusIP modbus_nodeRed;
ModBus_ESP32 MODBUS_GW(hostGW,&canalMODBUS_GW);

// Variáveis de controle
#define  BOMBAPIN 27
int modoPID = 0;                 // 0 - automático (PID), 1 - manual
int sampleTime = 100;
double PV_LDS01; // nivel lido pelo sensor LDS01
double PV_LDS02; // nivel lido pelo sensor LDS02
double SP = 80;
double MV; // tensaoPID
int tensaoPWM;
double KP = 1.6, KI = 0.15, KD = 0;
PID pidTank(&PV_LDS01,&MV,&SP,KP,KI,KD,DIRECT);


// Variaveis usadas para armazenar os dados referentes aos parametros dos links da rede
uint16_t Nlinks_LD01 = 0;
uint16_t Nlinks_LD02 = 0;
uint16_t Nlinks_TT05 = 0;
byte Address128[16];
uint8_t linkstatus = 0;
uint8_t signalQuality = 0;
int16_t signalStrength = 0;
uint32_t DPDUsTransmited = 0;
uint32_t DPDUsReceived = 0;
uint32_t DPDUsFailedTransmission = 0;
uint32_t DPDUsFailedReceived = 0;

// variáveis utilizadas como parametro da requisição via modbus dos valores de PV dos dois sensores
unsigned int adrrs_02 = 34;
unsigned int adrrs_01 = 13;
unsigned int len = 2;
byte ID = 1;
int aux = 1;
int res = 1;

// Endereço do dispositivo usado na requisição do Cód. 07 do padrão GSAP
byte GSAP_GNeighborHealthReport_Data_LDIS01[] = {0x00,0x08,0x4C,0x44,0x2D,0x49,0x53,0x2D,0x30,0x31,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte GSAP_GNeighborHealthReport_Data_LDIS02[] = {0x00,0x08,0x4C,0x44,0x2D,0x49,0x53,0x2D,0x30,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte GSAP_GNeighborHealthReport_Data_TTIS05[] = {0x00,0x08,0x54,0x54,0x2D,0x49,0x53,0x2D,0x30,0x35,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

// Variáveis utilizadas na Interrupção
volatile int contInterrup = 0; // utilizada pelo ISR para indicar a interrupção
int totalInterruptCounter;
hw_timer_t * timer = NULL; // define um ponteiro para uma estrutura do tipo hw_timer_t que o retorno da função "timerbegin()" usada para inicar o cronometro
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // usada nos processos críticos que envolvel "contInterrup"

// Função Interrupção
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  contInterrup++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Funções auxiliares
union Union2IntFloat{         // Função para conversão 2 Int para 1 Float
  short inteiros[2];
  float finalFloat;
};
Union2IntFloat x1;
Union2IntFloat x2;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Configuração com a rede WiFi
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //Inicializa o server modbus para a comunicação com o Node-Red
  modbus_nodeRed.server();

  // Abertura da sessão Gsap - (cód.01)
  int aux = 1;
  do{
    aux =canalGSAP.abrirCanalSessao(); // ABERTURA DO CANAL DE COMUNICAÇÃO E REQUISIÇÃO DA SESSÃO GSAP (COD.01)
  }while(aux);
  aux = 1;
  // Conexão Modbus + GW
  do{
    aux = MODBUS_GW.AbrirCanal();
  }while(aux);

  // Configuração do controlador
  pinMode(BOMBAPIN, OUTPUT);
  pidTank.SetTunings(KP,KI,KD);
  pidTank.SetMode(AUTOMATIC);
  pidTank.SetSampleTime(sampleTime);
 
  //Endereço de memória modbus para armazenamento do valor do Setpoint
  modbus_nodeRed.addHreg(11); // Valor do Setpoint

  // Mapeamento da memória Modbus para armazenamento dos parametros referentes aos dados do link com o LDS01
  modbus_nodeRed.addHreg(0); // LinksStatus
  modbus_nodeRed.addHreg(1); // SignalQuality
  modbus_nodeRed.addHreg(2); // SignalStrenght
  modbus_nodeRed.addHreg(3); // DPDUsTransmited
  modbus_nodeRed.addHreg(5); // DPDUsReceived
  modbus_nodeRed.addHreg(7); // DPDUsFailedTransmission
  modbus_nodeRed.addHreg(9); // DPDUsFailedReceived
  modbus_nodeRed.addHreg(12); // Número de links LDS01
  modbus_nodeRed.addHreg(13); // Valor de PV_LDS01
  modbus_nodeRed.addHreg(14); // Valor da tensão do sinal PWM enviado a bomba

  // Mapeamento da memória Modbus para armazenamento dos parametros referentes aos dados do link com o LDS02
  modbus_nodeRed.addHreg(15); // LinksStatus
  modbus_nodeRed.addHreg(16); // SignalQuality
  modbus_nodeRed.addHreg(17); // SignalStrenght
  modbus_nodeRed.addHreg(18); // DPDUsTransmited
  modbus_nodeRed.addHreg(20); // DPDUsReceived
  modbus_nodeRed.addHreg(22); // DPDUsFailedTransmission
  modbus_nodeRed.addHreg(24); // DPDUsFailedReceived
  modbus_nodeRed.addHreg(26); // Número de links LDS02
  modbus_nodeRed.addHreg(27); // Valor de PV_LDS02

  // Mapeamento da memória Modbus para armazenamento dos parametros referentes aos dados do segundo link com o LDS02
  modbus_nodeRed.addHreg(28); // LinksStatus
  modbus_nodeRed.addHreg(29); // SignalQuality
  modbus_nodeRed.addHreg(30); // SignalStrenght
  modbus_nodeRed.addHreg(31); // DPDUsTransmited
  modbus_nodeRed.addHreg(33); // DPDUsReceived
  modbus_nodeRed.addHreg(35); // DPDUsFailedTransmission
  modbus_nodeRed.addHreg(37); // DPDUsFailedReceived

  // Mapeamento da memória Modbus para armazenamento dos parametros referentes aos dados do segundo link com o TT05
  modbus_nodeRed.addHreg(39); // LinksStatus
  modbus_nodeRed.addHreg(40); // SignalQuality
  modbus_nodeRed.addHreg(41); // SignalStrenght
  modbus_nodeRed.addHreg(42); // DPDUsTransmited
  modbus_nodeRed.addHreg(44); // DPDUsReceived
  modbus_nodeRed.addHreg(46); // DPDUsFailedTransmission
  modbus_nodeRed.addHreg(48); // DPDUsFailedReceived
  modbus_nodeRed.addHreg(50); // Número de links do TT05
  
  // Inicializa variáveis da Interrupcao Timer
  timer = timerBegin(0, 80, true); // inicializar nosso cronometro - 80 é usado para indicar que o nosso cronometro eh incrementado 1,000,000 de vezes por seg
  timerAttachInterrupt(timer, &onTimer, true); // associa nosso cronometro a interrupção - true para indicar que a interrupção será do tipo borda
  timerAlarmWrite(timer,1000000, true); // define qndo o cronometro irá gerar um alarme para ser indicado uma interrupção  - 1000000 para cada seg - e true para resetar
  timerAlarmEnable(timer); // habilia nosso cronometro

}

void loop() {
  // put your main code here, to run repeatedly:
  modbus_nodeRed.task();

  if (contInterrup > 0) { // verifica se há uma interrupção a ser tratada
    // Coleta dos dados do link com o LDS01
    if (canalGSAP.Gsap07(GSAP_GNeighborHealthReport_Data_LDIS01) == 0){
        
        Nlinks_LD01 = canalGSAP.getNumLinks(); // obtem o número de links. É o número de índices disponiveis para ser usado com o ponteiro "NeighborHealthList"

        linkstatus = canalGSAP.NeighborHealthList[0].linkstatus;
        signalQuality = canalGSAP.NeighborHealthList[0].signalQuality;
        signalStrength = canalGSAP.NeighborHealthList[0].signalStrength;
        DPDUsTransmited = canalGSAP.NeighborHealthList[0].DPDUsTransmitted;
        DPDUsReceived = canalGSAP.NeighborHealthList[0].DPDUsReceived;
        DPDUsFailedTransmission = canalGSAP.NeighborHealthList[0].DPDUsFailedTransmission;
        DPDUsFailedReceived = canalGSAP.NeighborHealthList[0].DPDUsFailedReception;

        // Obrigatório da biblioteca
        canalGSAP.clean07();

        // Armazenamento do dados na memória mapeada anteriormente
        modbus_nodeRed.Hreg(0,linkstatus);
        modbus_nodeRed.Hreg(1,signalQuality);
        modbus_nodeRed.Hreg(2,~signalStrength+1);
        modbus_nodeRed.Hreg(3,DPDUsTransmited);
        modbus_nodeRed.Hreg(5,DPDUsReceived);
        modbus_nodeRed.Hreg(7,DPDUsFailedTransmission);
        modbus_nodeRed.Hreg(9,DPDUsFailedReceived);
        modbus_nodeRed.Hreg(12,Nlinks_LD01);
    }
    if (canalGSAP.Gsap07(GSAP_GNeighborHealthReport_Data_LDIS02) == 0){
        
        Nlinks_LD02 = canalGSAP.getNumLinks(); // obtem o número de links. É o número de índices disponiveis para ser usado com o ponteiro "NeighborHealthList"

        linkstatus = canalGSAP.NeighborHealthList[0].linkstatus;
        signalQuality = canalGSAP.NeighborHealthList[0].signalQuality;
        signalStrength = canalGSAP.NeighborHealthList[0].signalStrength;
        DPDUsTransmited = canalGSAP.NeighborHealthList[0].DPDUsTransmitted;
        DPDUsReceived = canalGSAP.NeighborHealthList[0].DPDUsReceived;
        DPDUsFailedTransmission = canalGSAP.NeighborHealthList[0].DPDUsFailedTransmission;
        DPDUsFailedReceived = canalGSAP.NeighborHealthList[0].DPDUsFailedReception;

        // Armazenamento do dados na memória mapeada anteriormente
        modbus_nodeRed.Hreg(15,linkstatus);
        modbus_nodeRed.Hreg(16,signalQuality);
        modbus_nodeRed.Hreg(17,~signalStrength+1);
        modbus_nodeRed.Hreg(18,DPDUsTransmited);
        modbus_nodeRed.Hreg(20,DPDUsReceived);
        modbus_nodeRed.Hreg(22,DPDUsFailedTransmission);
        modbus_nodeRed.Hreg(24,DPDUsFailedReceived);
        modbus_nodeRed.Hreg(26,Nlinks_LD02);

        linkstatus = canalGSAP.NeighborHealthList[1].linkstatus;
        signalQuality = canalGSAP.NeighborHealthList[1].signalQuality;
        signalStrength = canalGSAP.NeighborHealthList[1].signalStrength;
        DPDUsTransmited = canalGSAP.NeighborHealthList[1].DPDUsTransmitted;
        DPDUsReceived = canalGSAP.NeighborHealthList[1].DPDUsReceived;
        DPDUsFailedTransmission = canalGSAP.NeighborHealthList[1].DPDUsFailedTransmission;
        DPDUsFailedReceived = canalGSAP.NeighborHealthList[1].DPDUsFailedReception;
        
        // Armazenamento do dados na memória mapeada anteriormente (Parametros do segundo link do LD02)
        modbus_nodeRed.Hreg(28,linkstatus);
        modbus_nodeRed.Hreg(29,signalQuality);
        modbus_nodeRed.Hreg(30,~signalStrength+1);
        modbus_nodeRed.Hreg(31,DPDUsTransmited);
        modbus_nodeRed.Hreg(33,DPDUsReceived);
        modbus_nodeRed.Hreg(35,DPDUsFailedTransmission);
        modbus_nodeRed.Hreg(37,DPDUsFailedReceived);

        // Obrigatório da biblioteca
        canalGSAP.clean07();
    }
    if (canalGSAP.Gsap07(GSAP_GNeighborHealthReport_Data_TTIS05) == 0){
        
        Nlinks_TT05 = canalGSAP.getNumLinks(); // obtem o número de links. É o número de índices disponiveis para ser usado com o ponteiro "NeighborHealthList"

        linkstatus = canalGSAP.NeighborHealthList[0].linkstatus;
        signalQuality = canalGSAP.NeighborHealthList[0].signalQuality;
        signalStrength = canalGSAP.NeighborHealthList[0].signalStrength;
        DPDUsTransmited = canalGSAP.NeighborHealthList[0].DPDUsTransmitted;
        DPDUsReceived = canalGSAP.NeighborHealthList[0].DPDUsReceived;
        DPDUsFailedTransmission = canalGSAP.NeighborHealthList[0].DPDUsFailedTransmission;
        DPDUsFailedReceived = canalGSAP.NeighborHealthList[0].DPDUsFailedReception;

        // Obrigatório da biblioteca
        canalGSAP.clean07();

        // Armazenamento do dados na memória mapeada anteriormente
        modbus_nodeRed.Hreg(39,linkstatus);
        modbus_nodeRed.Hreg(40,signalQuality);
        modbus_nodeRed.Hreg(41,~signalStrength+1);
        modbus_nodeRed.Hreg(42,DPDUsTransmited);
        modbus_nodeRed.Hreg(44,DPDUsReceived);
        modbus_nodeRed.Hreg(46,DPDUsFailedTransmission);
        modbus_nodeRed.Hreg(48,DPDUsFailedReceived);
        modbus_nodeRed.Hreg(50,Nlinks_TT05);
    }
    // Requisição dos valores de PV e cálculo do controle com base no valor de PV_LDS01
    res = MODBUS_GW.ModBus_Request04(adrrs_01,len,ID,x1.inteiros);
    if(res==0){
      PV_LDS01 = (double)(x1.finalFloat);  // valor retornado pelo GW
      // Armazenamento do valor de PV_LDS01 no REG na memória modbus
      modbus_nodeRed.Hreg(13,PV_LDS01);
      SP = modbus_nodeRed.Hreg(11);
      Serial.println(PV_LDS01);
      Serial.println(SP);
      // Cálculo do controle
      pidTank.Compute();
      // Armazenamento do valor de MV no REG na memória modbus
      modbus_nodeRed.Hreg(14,MV);
      // Converte o valor de de MV para o valor da tensão de saída do ESP32
      tensaoPWM = (int)MV;
      // Mapeamento dos valores de tensão
      tensaoPWM = (int)map(tensaoPWM, 0, 255, 120, 254);
      // Saída da tensão
      analogWrite(BOMBAPIN, tensaoPWM); 
      }
    res = 1;
    // Requisição do valor de PV obtido apartir do sensor LD_02
    res = MODBUS_GW.ModBus_Request04(adrrs_02,len,ID,x2.inteiros);
    if(res==0){
      PV_LDS02 = (double)(x2.finalFloat);  // valor retornado pelo GW
      // Armazenamento do valor de PV_LDS01 no REG na memória modbus
      modbus_nodeRed.Hreg(27,PV_LDS02);
      Serial.println(PV_LDS02);
    }
    portENTER_CRITICAL(&timerMux);
    contInterrup--; // a interrupção detectada foi tratada
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;
    Serial.print("An interrupt as occurred. Total number: ");
    Serial.println(totalInterruptCounter);
    Serial.println("Eh esse");
  }
}
