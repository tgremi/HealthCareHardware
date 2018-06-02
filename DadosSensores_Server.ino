#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

/*
 * String com numero de série do dispositivo, para identificar a qual usuário pertence.
 */
static String HDW = "HDWPIVIIS1";
MPU6050 sensor;
/* 
 * Valores puros que serão recebidos do sensor, utilizamos int16_t para um int com 16 bits, tamanho maximo recebido 
 *  pelo ESP8266, quando utilizado pela IDE do arduino. 
 */
int16_t ax, ay, az;
int16_t gx, gy, gz;

long tempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

void setup()
{

  /* 
  * Iniciando comunicação Serial do dispositivo com Bade Rate de 115200
  */
  Serial.begin(115200);

  /* 
  * Iniciando conexão Wi-fi
  */
  WiFi.begin("SSID", "NETWORK_PASS");

  /* 
  * Testando conexão 
  */
  while (WiFi.status() != WL_CONNECTED)
  {
    /* 
  * Aguarda conexão
  */
    delay(500);
    Serial.println("Aguardando conexão");
  }

  /* 
  * Inicia a comunicação I2C com o sensor, inicializando as portas D6 e D7 do Esp8266
  */

  Wire.begin(D6, D7);
  Wire.beginTransmission(0X68);

  /* 
  * Inicialica o sensor MPU6050
  */
  sensor.initialize();
  Serial.println("Dispositivo conectado!");

  /* 
  * Testando inicialização do sensor. 
  */
  if (sensor.testConnection())
    Serial.println("Sensor iniciado Corretamente");
}

/* 
  * Função responsável por montar o arquivo e enviar para o server no formato JSON
  */
void sendToServer(float ax, float ay, float az, float rx, float ry)
{

  StaticJsonBuffer<300> JSONbuffer; //Declaring static JSON buffer
  JsonObject &JSONencoder = JSONbuffer.createObject();

  JSONencoder["hardware_number"] = HDW;
  JSONencoder["ax"] = ax;
  JSONencoder["ay"] = ay;
  JSONencoder["az"] = az;
  JSONencoder["rx"] = rx;
  JSONencoder["ry"] = ry;

  char JSONmessageBuffer[300];
  JSONencoder.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

  HTTPClient http;

  http.begin("http://sheltered-tor-84466.herokuapp.com/esp8266"); // <-- Endpoint que receberá os dados e irá analisa-los
  http.addHeader("Content-Type", "application/json");             // <-- Tipo de dado que esta sendo enviado para o servidor.

  int httpCode = http.POST(JSONmessageBuffer); // <--  Enviando a requisição tipo POST
  String payload = http.getString();           // <-- Recebe a resposta do servidor

  // Serial.println(httpCode);   //Print HTTP return code
  //Serial.println(payload);    //Print request response payload

  http.end(); //<-- encerra a conexão
}

void loop()
{

  if (WiFi.status() == WL_CONNECTED)
  {

    // Lendo as acelerações e velocidades angulares
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);

    dt = (millis() - tempo_prev) / 1000.0;
    tempo_prev = millis();

    // Calculando as acelerações nos eixoz X, Y e Z, conforme o sensor se move, as acelerações mudam para cada eixo.
    float ax_m_s2 = ax * (9.81 / 16384.0);
    float ay_m_s2 = ay * (9.81 / 16384.0);
    float az_m_s2 = az * (9.81 / 16384.0);

    // Calcula os angulos com o acelerometro
    float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
    float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

    //Calcula os angulos com giroscopios implementando o filtro complementar
    ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
    ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;

    ang_x_prev = ang_x;
    ang_y_prev = ang_y;

    Serial.print("RX:  ");
    Serial.print(ang_x);
    Serial.print("\t RY: ");
    Serial.println(ang_y);
    Serial.println("");
    Serial.print("ax ");
    Serial.print(ax_m_s2);
    Serial.print("; ");
    Serial.print("ay ");
    Serial.print(ay_m_s2);
    Serial.print("; ");
    Serial.print("az ");
    Serial.print(az_m_s2);
    Serial.println("; ");

    // Envia os dados dos sensores para o servidor... 
    sendToServer(ax_m_s2, ay_m_s2, az_m_s2, 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x, 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y);

    delay(100);
  }
  else
  {

    Serial.println("Wifi não conectado!");
  }

  //elay(1);  //Send a request every 30 seconds
}
