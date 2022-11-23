/*
 *  Projeto financiado pelo Instituto Federal Sul-Rio-Grandense
 *  Desenvolvedores : Laisa / Marcelo Kwecko
 *  Placa: ESP32­WROOM­32D
 *  IDE 1.8.X
 *  Versao da Placa ESP32 2.0.4
 */ 

/*  Para a Compilação: 
    Placa: DOIT ESP32 DEVKIT V1
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include "esp_task_wdt.h"   /* Watchdog */  

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

/* Variaveis para determinacao do Sp02*/
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

 /* Habilita ou nao o modo DEBUG */
bool DEBUG=true;

/* Armazena o Valor inicial de IR na inicializacao */
long unblockedValue; 

//byte pulseLED = 11; //Must be on PWM pin
/* LED que indica leitura dos valores*/
byte StatusLED = 2; 

/* Minutos do WatchDog */
int T_WATCHDOG = 3;
bool LED_BLINK = false;   /* Controla a sinalizacao ou nao no LED */
bool bFINGER = false;     /* Determina se teve a troca do dedo */

/* Outra TASK */
TaskHandle_t Task2Core;

float fTEMP;    /* Armazena a temp. lida no sensor */


/* ***************************************************************** */
/* Funcoes generica                                                  */
/* ***************************************************************** */

// Função que sinaliza no LED uma acao
// **************************************************
void BlinkLed(int t = 100)
{
  digitalWrite(StatusLED, !digitalRead(StatusLED));
  delay(t);
  digitalWrite(StatusLED, !digitalRead(StatusLED));
}

// Funcoes de inicializa o Sensor                                    
// *****************************************************************
void SensorSetup()
{
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings  
  particleSensor.setPulseAmplitudeRed(0); //Turn off Red LED
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}


/* ***************************************************************** */
/* Funcoes que informa se ha um dedo no sensor                       */
/* ***************************************************************** */
bool CheckFinger()
{
// Reinicializa os parametros do Sensor
  SensorSetup();
  long currentDelta = particleSensor.getIR() - unblockedValue;

  // Identifica a presença de um dedo
  if (currentDelta > (long)100)  
    return true; 
  else  
    return false;
 
}

/* ***************************************************************** */
/* Funcoes de leitura da temperatura                                 */
/* ***************************************************************** */
float Temperatura()
{
   particleSensor.setup(0); //Configure sensor. Turn off LEDs
   particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

   float TEMP = particleSensor.readTemperature();

   for (int i=0;i<=100; i++)
      TEMP = (TEMP + particleSensor.readTemperature()) / 2;
   
   return TEMP;    
}


/* ***************************************************************** */
/* Funcoes de leitura da SPO2 e Batimentos                           */
/* ***************************************************************** */
void SPO2_HR()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  int iCount = 0;
  int iReadTrue = 0;

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  /* Configura o Sensor */
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  do {
        // read the first 100 samples, and determine the signal range
        for (byte i = 0 ; i < bufferLength ; i++)
        {
            while (particleSensor.available() == false) //do we have new data?
            particleSensor.check(); //Check the sensor for new data

            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample(); //We're finished with this sample so move to next sample

            //Serial.print(F("red="));
            //Serial.print(redBuffer[i], DEC);
            //Serial.print(F(", ir="));
            //Serial.print(irBuffer[i], DEC);
          }

        //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
        
        if ((validSPO2 == 0) or (validHeartRate == 0))
           iCount += 1;
        else 
           iReadTrue += 1;

        if (DEBUG == true) {
            Serial.print(F("\n[DEBUG] HR="));
            Serial.print(heartRate, DEC);

            Serial.print(F(", HRvalid="));
            Serial.print(validHeartRate, DEC);

            Serial.print(F(", SPO2="));
            Serial.print(spo2, DEC);

            Serial.print(F(", SPO2Valid="));
            Serial.println(validSPO2, DEC);
        }

  } while((iCount < 10) and (iReadTrue < 4));


  if (iCount == 10) {

      Serial.print(F(" - HR= 0"));
      Serial.println(F(" - SPO2= 0"));
    
  } else {

    Serial.print(F(" - HR="));
    Serial.print(heartRate, DEC);
    Serial.print(F(" - SPO2="));
    Serial.println(spo2, DEC);
  
  }
  
}


/* ***************************************************************** */
/* Funcao - 2 TASK                                                   */
/* ***************************************************************** */

// Função executada no segundo Core
/*********************************/
void Task_2Core( void * parameter ) {

 Serial.print("[TASK 2] Informaçao de Identificacao de uma Leitura no Sensor! : Core ");
 Serial.println(xPortGetCoreID());
 delay(1000);
 
 for(;;){
    
    // Reseta o temporizador do watchdog
    esp_task_wdt_reset();

    // Sinaliza a identificacao do dedo no sensor
    if (LED_BLINK == true) 
        BlinkLed();

    delay(100);
 }

}


/* ***************************************************************** */
/* Funcao de Inicializacao da placa                                  */
/* ***************************************************************** */
void setup()
{
  Serial.begin(115200); // Iicializa a porta de comunicação serial 

  pinMode(StatusLED, OUTPUT);       /* Configura o pino do LED */
  digitalWrite(StatusLED, HIGH);    /* Sinaliza que a placa esta no modo de inicialização */
  
  // Determinda se o ESP foi inicializado via WatchDog ou não 
  if ( (esp_reset_reason() == ESP_RST_WDT) || (esp_reset_reason() == ESP_RST_TASK_WDT) ) {
      Serial.println("[WARNING] Rebooting via WatchDog!");  
  } else if (esp_reset_reason() == ESP_RST_PANIC) {    // Reset via Kernel Panic
            Serial.println("[WARNING] Rebooting devido a Panic!");
          } else if (esp_reset_reason() == ESP_RST_SW ) {      // Reset via software
              Serial.println("[WARNING] Rebooting devido a Restart!");
          } else {
              Serial.println("Power On!");
          }
          
   
  // Definir a Freq. de Trabalho; 
  // Ao comentar essa linha freq de OP é fixada em 240 MHz
  //setCpuFrequencyMhz(80);

   /* Inpmrime informações do Chip */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.printf("Chip ESP32 - %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    Serial.printf("silicon revision %d, ", chip_info.revision);

    Serial.printf("%dMB %s flash, CPU Freq: %s MHz\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external", String(ESP.getCpuFreqMHz()));


   /* Inicializa o Sensor */ 
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("[ERRO] MAX30105 não foi encontrado. Por favor verifique as conexões e o sensor!"));
    while (1); /* Trava o sistema; Necessita ser reinicalizado */
  }

  Serial.println(F("[WARNING] Sensor encontrado!"));

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.setPulseAmplitudeRed(0); //Turn off Red LED
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //Take an average of IR readings at power up
  unblockedValue = 0;
  for (byte x = 0 ; x < 32 ; x++)
  {
    unblockedValue += particleSensor.getIR(); //Read the IR value
  }
  unblockedValue /= 32;

  /* Criacao da task que executa funções no segundo core */ 
  xTaskCreatePinnedToCore(
                    Task_2Core,       /* Funcao ao ser executada. */
                    "Task 2 Core",    /* Nome da task . */
                    2048,             /* Tamanho da Stack para variaveis Bytes */ 
                    NULL,             /* Parametro para a Task */
                    5,                /* Prioridade. Maior, mais prioridade */
                    &Task2Core,       /* Task handle to keep track of created task */
                    APP_CPU_NUM);     /* APP_CPU_NUM = 1 e PRO_CPU_NUM 0 */

  /* WatchDog */ 
  esp_task_wdt_init(T_WATCHDOG * 60, true);
  esp_task_wdt_add(NULL);
  esp_task_wdt_add(Task2Core); /* Adiciona a segunda Task */

  /* Executa a funcao a cada segundo */
  //CRON1.start();
  
  digitalWrite(StatusLED, LOW); /* Sinaliza que finalizou o modo de inicialização */
  Serial.println(F("Sistema inicializado. Coloque o dedo no sensor!"));
  
}


/* ***************************************************************** */
/* Funcao Loop principal                                             */
/* ***************************************************************** */
void loop()
{
  //Reseta o temporizador do watchdog
  esp_task_wdt_reset();

  // Identifica a presença de um dedo
  if ((CheckFinger()) and (!bFINGER))
  {
    
    (!LED_BLINK) ? Serial.println("[WARNING] Dedo identificado!") : NULL;
    LED_BLINK = true; /* sinaliza um nova leitura no led */
    bFINGER=true;     /* registra que há um dedo no sensor */

    // Leitura da Temperatura
    fTEMP = CheckFinger() == true ? Temperatura() : 0;
    Serial.print("Temperatura: ");
    Serial.print(fTEMP, 2);
    Serial.print(" °C");

    // Leitura da SPO2 e Batimentos
    if ((CheckFinger()) and (fTEMP != 0))
    {
      // Leitura da Oxigenação e Batimentos Cardiaco
      SPO2_HR(); 
    }

    /* Informa o fim da leitura no led */
    LED_BLINK = false;
    delay(1000);
  }

  // Habilita uma nova leitura para outro dedo 
  if (!CheckFinger())
  {
     bFINGER = false; /* Registra que o dedo nao esta no sensor; Permite nova leitura*/
     delay(1000);
  }

  delay(10);   
}
