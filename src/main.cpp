#include "Arduino.h"
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h> //Biblioteca para as publicações via mqtt
#include "mqtt_client.h"  //Arquivo com as funções de mqtt
#include "MAX30105.h"
#include "heartRate.h"
//------------------------------------Definições de rede-----------------------------
#define WIFISSID "trator"                                    // Coloque seu SSID de WiFi aqui
#define PASSWORD "Pauloh01"                                  // Coloque seu password de WiFi aqui
#define TOKEN "BBFF-rX9QpMCCnyo6oNUW0xErfP0e7ixvjJ"          // Coloque seu TOKEN do Ubidots aqui
#define variable_label_spo2 "Saturação de Oxigenio"          // Label referente a variável de temperatura criada no ubidots
#define VARIABLE_LABEL_HEAT_RATE "Frequencia Cardiaca"       // Label referente a variável de umidade criada no ubidots
#define DEVICE_ID "BBFF-fc2c3213408ddcd65448b2fc1996251f155" // ID do dispositivo (Device id, também chamado de client name)
#define SERVER "things.ubidots.com"                          // Servidor do Ubidots (broker)
#define PORT 1883                                            // Porta padrão

// Tópico aonde serão feitos os publish, "esp32-dht" é o DEVICE_LABEL
#define TOPIC "/v1.6/devices/CPAP_RESPIRE"

WiFiClient ubidots;           // Objeto WiFiClient usado para a conexão wifi
WiFiServer sv(80);
PubSubClient client(ubidots); // Objeto PubSubClient usado para publish–subscribe
MAX30105 particleSensor;
QueueHandle_t fila;

struct
{
  double bpm;
  double espo2;
} typedef mqtt_dados_t;

#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 93.0      // 88.0      //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5      // if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 10.0

const char* ssid = "ESP32-AP"; //Define o nome do ponto de acesso Access Point
const char* pass = "12345678"; //Define a senha

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
double beatAvg;
//---------------------------------Funções--------------------------------------------------

void reconnect();
bool mqttInit();
bool sendValues(double bpm, double espo2);
void mqtt_task(void *pvt);
void geraAP();

//---------------------------------SETUP----------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

 //---------------------------------------- Configurando AP------------------------------------------------
  Serial.println("\n"); //Pula uma linha                                                                  |
  WiFi.softAP(ssid, pass); //Inicia o ponto de acesso                                                     |
  Serial.print("Se conectando a: "); //Imprime mensagem sobre o nome do ponto de acesso                   |
  Serial.println(ssid); //                                                                                |
  IPAddress ip = WiFi.softAPIP(); //Endereço de IP                                                        |
  Serial.print("Endereço de IP: "); //Imprime o endereço de IP                                            |
  Serial.println(ip);               //                                                                    |
  sv.begin(); //Inicia o servidor                                                                         |
  Serial.println("Servidor online"); //Imprime a mensagem de início                                       |
  //-------------------------------------------------------------------------------------------------------
  if (!mqttInit())
  {
    Serial.println("Failed!");
    ESP.restart();
  }
  Serial.println("MQTT -> Conectado");

  fila = xQueueCreate(10, sizeof(mqtt_dados_t));

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  byte ledBrightness = 0x0A; // Options: 0=Off to 255=50mA
  byte sampleAverage = 8;    // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;          // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 1000;      // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215;       // Options: 69, 118, 215, 411
  int adcRange = 2048;       // Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
  // particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  // particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  xTaskCreatePinnedToCore(mqtt_task, "MQTT_TASK", 2048, NULL, 1, NULL, 1);
}
//------------------------------------------------------------------------------------
int x = 1000000;
int y = 30;
uint32_t ir, red;
float red_forGraph = 0.0;
float ir_forGraph = 0.0;
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100; // calculate SpO2 by this sampling interval
double fred, fir;
double SpO2 = 0;
double ESpO2 = 95.0; // initial value of estimated SpO2
double FSpO2 = 0.7;  // filter factor for estimated SpO2
double frate = 0.95; // low pass filter for IR/red LED value to eliminate AC component

void loop()
{
//--------------------------------------------------gerando a rede-------------------------------------------
geraAP(); // cria Access Point
//-----------------------------------------------------------------------------------------------------------

// Checagem de Batimento Cardíaco
long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; 
        rateSpot %= RATE_SIZE;                    

        // Take average of readings
        beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++)
            beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
       }
   }
  //Serial.print("Fila IR=");
  //Serial.print(ir);
  //Serial.print(", fila RED=");
  //Serial.print(red);
  //Serial.print(", IR=");
  //Serial.print(irValue);
  //Serial.print(", BPM=");
  //Serial.print(beatsPerMinute);
  //Serial.print(", Avg BPM=");
 // Serial.println(beatAvg);

  if (irValue < 50000)
    Serial.println(" No finger?");

  Serial.println();
  while (particleSensor.available())
  {                                   // do we have new data
    red = particleSensor.getFIFOIR(); // why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); // why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
    i++;
    //Serial.print("i = ");
    //Serial.print(i);
    fred = (double)red;
    fir = (double)ir;
    //Serial.print("| fred = ");
    //Serial.print(red);
    //Serial.print("fir = ");
    //Serial.print(ir);
    avered = avered * frate + (double)red * (1.0 - frate); // average red level by low pass filter
    //Serial.print("| zavered = ");
    //Serial.print(avered);
    aveir = aveir * frate + (double)ir * (1.0 - frate);    // average IR level by low pass filter
    //Serial.print("aveir = ");
    //Serial.print(aveir);
    sumredrms += (fred - avered) * (fred - avered);        // square sum of alternate component of red level
    Serial.print("|sumredrms +=");
   // Serial.print(sumredrms);
    sumirrms += (fir - aveir) * (fir - aveir);             // square sum of alternate component of IR level
    //Serial.print("sumirrms += ");
    //Serial.print(sumirrms);
    if ((i % SAMPLING) == 0)
    { // slow down graph plotting speed for arduino Serial plotter by thin out
      if (millis() > TIMETOBOOT)
      {
        ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        // trancation for Serial plotter's autoscaling
        if (ir_forGraph > x)
          ir_forGraph = x;
        if (ir_forGraph < y)
          ir_forGraph = y;
        if (red_forGraph > x)
          red_forGraph = x;
        if (red_forGraph < y)
          red_forGraph = y;
        //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
        if (ir < FINGER_ON)
          // ESpO2 = MINIMUM_SPO2;    //indicator for finger detached
          Serial.println();
        Serial.print(" -> ");
       // Serial.print(ir_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(" -> ");
       // Serial.print(red_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(" -> ");
       Serial.print(sumredrms); // low pass filtered SpO2
      }
    }
    if ((i % Num) == 0)
    {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      SpO2 = -23.3 * (R - 0.4) + 100;
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // passa baixa
      sumredrms = 0.0;
      sumirrms = 0.0;
      i = 0;
      break;
    }
    particleSensor.nextSample(); // We're finished with this sample so move to next sample
    //Serial.print(", BPM->");
   // Serial.print(beatAvg);
    //Serial.print(", SpO2->");
   // Serial.println(SpO2);

    mqtt_dados_t dados;
    dados.bpm = beatAvg;
    dados.espo2 = SpO2;
     // cria Access Point
 //-----------------------------------------------------------------------------------------------------------------------------------------------------
WiFiClient client = sv.available(); //Cria o objeto cliente
  if (client) { //Se este objeto estiver disponível
    String line = ""; //Variável para armazenar os dados recebidos
    while (client.connected()) { //Enquanto estiver conectado
      if (client.available()) { //Se estiver disponível
        char c = client.read(); //Lê os caracteres recebidos
        if (c == '\n') { //Se houver uma quebra de linha
          if (line.length() == 0) { //Se a nova linha tiver 0 de tamanho
            client.println("HTTP/1.1 200 OK"); //Envio padrão de início de comunicação
            client.println("Content-type:text/html");
            client.println();
            client.print("<hr />"); 
            client.print("<h1 style=text-align:center><strong>RESPIRE</strong></h1>");
            client.print("<hr />"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("Ligue o led clicando <a href=\"/ligar\">aqui</a><br>"); //Linha para ligar o led
            client.print("Desligue o led clicando <a href=\"/desligar\">aqui</a><br>"); //Linha para desligar o led
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<h1 style=text-align:center><span style=font-size:28px><span style=color:#0000FF><strong>Satura&ccedil;&atilde;o de Oxig&ecirc;nio</strong></span></span></h1>"); 
            client.print("<p style=text-align:center><strong>X</strong></p> %f.");
            client.print(dados.espo2); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<h1 style=text-align:center><span style=font-size:28px><span style=color:#FF0000><strong>Batimentos Card&iacute;acos</strong></span></span></h1>"); 
            client.print("<p style=text-align:center><strong></strong></p>" );
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print(dados.bpm);
            client.print("<hr />");
            client.println();
            break;
          } else {   
            line = "";
          }
        } else if (c != '\r') { 
          line += c; //Adiciona o caractere recebido à linha de leitura
        }
        if (line.endsWith("GET /ligar")) { //Se a linha terminar com "/ligar", liga o led
          digitalWrite(23, HIGH);               
        }
        if (line.endsWith("GET /desligar")) { //Se a linha terminar com "/desligar", desliga o led
          digitalWrite(23, LOW);              
        }
      }
    }
    client.stop(); //Para o cliente
  }

 //-----------------------------------------------------------------------------------------------------------------------------------------------------
   // xQueueSend(fila, &dados, portMAX_DELAY);
    //delay(200);                  //----------------TEMPO DE ENVIO
    //Serial.print("======");
    //Serial.print(SpO2);
  }
}
bool mqttInit()
{
  // Inicia WiFi com o SSID e a senha
  WiFi.begin(WIFISSID, PASSWORD);

  // Loop até que o WiFi esteja conectado
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }

  // Exibe no monitor serial
  Serial.println("Connected to network");

  // Seta servidor com o broker e a porta
  client.setServer(SERVER, PORT);

  // Conecta no ubidots com o Device id e o token, o password é informado como vazio
  while (!client.connect(DEVICE_ID, TOKEN, ""))
  {
    Serial.println("MQTT - Connect error");
    return false;
  }

  Serial.println("MQTT - Connect ok");
  return true;
}

void reconnect()
{
  // Loop até que o MQTT esteja conectado
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");

    // Tenta conectar
    if (client.connect(DEVICE_ID, TOKEN, ""))
      Serial.println("connected");
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());

      Serial.println(" try again in 2 seconds");
      // Aguarda 2 segundos antes de retomar
      delay(2000);
    }
  }
}

bool sendValues(double bpm, double espo2)
{
  char json[250];

  // Atribui para a cadeia de caracteres "json" os valores referentes a temperatura e os envia para a variável do ubidots correspondente
  sprintf(json, "{\"%s\":{\"value\":%02.02f}}", variable_label_spo2, espo2);

  if (!client.publish(TOPIC, json))
    return false;

  // Atribui para a cadeia de caracteres "json" os valores referentes a umidade e os envia para a variável do ubidots correspondente
  sprintf(json, "{\"%s\":{\"value\":%02.02f}}", VARIABLE_LABEL_HEAT_RATE, bpm);

  if (!client.publish(TOPIC, json))
    return false;

  // Se tudo der certo retorna true
  return true;
  Serial.println("* PUBLICADO ** ");
}

void beatsminute_task(void *pvt)
{
 // bpm_dados_task dados;
  
}


void mqtt_task(void *pvt)
{

  mqtt_dados_t dados;

  while (1)
  {
    reconnect();
    while (xQueueReceive(fila, &dados, 100 * portTICK_PERIOD_MS))
    {
      sendValues(dados.bpm, dados.espo2);
    }
  }
}

void geraAP(){
  WiFiClient client = sv.available(); //Cria o objeto cliente
  if (client) { //Se este objeto estiver disponível
    String line = ""; //Variável para armazenar os dados recebidos
    while (client.connected()) { //Enquanto estiver conectado
      if (client.available()) { //Se estiver disponível
        char c = client.read(); //Lê os caracteres recebidos
        if (c == '\n') { //Se houver uma quebra de linha
          if (line.length() == 0) { //Se a nova linha tiver 0 de tamanho
            client.println("HTTP/1.1 200 OK"); //Envio padrão de início de comunicação
            client.println("Content-type:text/html");
            client.println();
            client.print("<hr />"); 
            client.print("<h1 style=text-align:center><strong>RESPIRE</strong></h1>");
            client.print("<hr />"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<h1 style=text-align:center><span style=font-size:28px><span style=color:#0000FF><strong>Satura&ccedil;&atilde;o de Oxig&ecirc;nio</strong></span></span></h1>"); 
            client.print("<p style=text-align:center><strong>X</strong></p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<h1 style=text-align:center><span style=font-size:28px><span style=color:#FF0000><strong>Batimentos Card&iacute;acos</strong></span></span></h1>"); 
            client.print("<p style=text-align:center><strong>Y</strong></p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<p>&nbsp;</p>"); 
            client.print("<p>&nbsp;</p>");
            client.print("<hr />");
            client.println();
            break;
          } else {   
            line = "";
          }
        } else if (c != '\r') { 
          line += c; //Adiciona o caractere recebido à linha de leitura
        }
        if (line.endsWith("GET /ligar")) { //Se a linha terminar com "/ligar", liga o led
          digitalWrite(23, HIGH);               
        }
        if (line.endsWith("GET /desligar")) { //Se a linha terminar com "/desligar", desliga o led
          digitalWrite(23, LOW);              
        }
      }
    }
    client.stop(); //Para o cliente
  }
} 