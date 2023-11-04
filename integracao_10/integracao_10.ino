#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include <WiFi.h>
#include "time.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

const int numMeasurements = 10; // Número de medidas para calcular a média
int heartRateMeasurements[numMeasurements]; // Array para armazenar as medidas
int currentMeasurement = 0; // Índice da medida atual
int measurementCount = 0;
int averageHeartRate;

const char* ssid       = "OsirMaxCaroline";
const char* password   = "a1s2d3f4";

const char* ntpServer = "br.pool.ntp.org";
const long  gmtOffset_sec = -(3600 * 3);
const int   daylightOffset_sec = 0;

String getDateTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
  }
  char buffer[30];
  // Obter o tempo atual em milissegundos
  unsigned long current_millis = millis();
  snprintf(buffer, 30, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, current_millis % 1000);
  return String(buffer);
}

String getId() {
  uint64_t mac = ESP.getEfuseMac();
  String macStr = "";
  for (int i = 5; i >= 0; i--) {
    macStr += String((mac >> (8 * i)) & 0xFF, HEX);
    if (i > 0) {
      macStr += "";
    }
  }
  return macStr;
}

void postFallToServer(){

  Serial.println("Posting JSON data to server...");
  // Block until we are able to connect to the WiFi access point
  
  HTTPClient http;   
    
  // http.begin("https://atech-main-api-dev.onrender.com/api/wearables/insertData");  
  http.begin("http://192.168.8.102:3000/data");
  http.addHeader("Content-Type", "application/json");         
  
  StaticJsonDocument<200> doc;

  JsonObject obj = doc.createNestedObject();

  // Serial.printf("\nCHIP MAC: %012llx\n", ESP.getEfuseMac());
  
  // create an object
  JsonObject object = doc.to<JsonObject>();
  object["physical_id"] = getId();
  object["timestamp"] = getDateTime();
  object["event"] = "fall";
    
  String requestBody;
  serializeJson(doc, requestBody);
    
  int httpResponseCode = http.POST(requestBody);

  if(httpResponseCode>0){
      
    String response = http.getString();                       
      
    Serial.println(httpResponseCode);   
    Serial.println(response);
    
  }
  else {
    
    Serial.printf("Error occurred while sending HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
      
  }
     
}

void postDataToServer(float temperature, int32_t heartRate, int32_t spo2, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ) {
 
  Serial.println("Posting JSON data to server...");
  // Block until we are able to connect to the WiFi access point
  
  HTTPClient http;   
    
  // http.begin("https://atech-main-api-dev.onrender.com/api/wearables/insertData");  
  http.begin("http://192.168.8.102:3000/data");
  http.addHeader("Content-Type", "application/json");         
  
  StaticJsonDocument<200> doc;

  JsonObject obj = doc.createNestedObject();

  // Serial.printf("\nCHIP MAC: %012llx\n", ESP.getEfuseMac());
  
  // create an object
  JsonObject object = doc.to<JsonObject>();
  object["physical_id"] = getId();
  object["timestamp"] = getDateTime();
  object["temperature"] = temperature;
  object["heartRate"] = heartRate;
  object["spo2"] = spo2;
  object["accX"] = accX;
  object["accY"] = accY;
  object["accZ"] = accZ;
  object["gyrX"] = gyrX;
  object["gyrY"] = gyrY;
  object["gyrZ"] = gyrZ;
    
  String requestBody;
  serializeJson(doc, requestBody);
    
  int httpResponseCode = http.POST(requestBody);

  if(httpResponseCode>0){
      
    String response = http.getString();                       
      
    Serial.println(httpResponseCode);   
    Serial.println(response);
    
  }
  else {
    
    Serial.printf("Error occurred while sending HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
      
  }
     
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 25; //brilho do led ( 0 a 255)
  byte sampleAverage = 4; //Define a média de amostras para cada leitura
  byte ledMode = 2; //Define o modo de detecção ativa dos LEDs
  byte sampleRate = 250; //Define a taxa de amostragem como 100 amostras por segundo
  int pulseWidth = 215; // Define a largura de pulso como 69
  int adcRange = 4096; //Define a faixa do ADC como 4096

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY(); // Enable the temp ready interrupt. This is required.
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcular a aceleração total
  float totalAcceleration = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  float verticalAcceleration = a.acceleration.z;

  float thresholdTotalAcceleration = 10.0; // Ajuste esse valor conforme necessário

  float thresholdVerticalAcceleration = 13; // Ajuste esse valor conforme necessário

  // Verificar se a aceleração total excede o limite
  if (totalAcceleration > thresholdTotalAcceleration) {
    // Verificar se a aceleração vertical também excede o limite
    if (totalAcceleration > thresholdTotalAcceleration && verticalAcceleration > thresholdVerticalAcceleration) {
      // Queda detectada
      Serial.println("Queda detectada!");
      postFallToServer();
      // Serial.println(totalAcceleration);
      // Você pode tomar ações aqui, como enviar um aviso.
    } else {
      // Movimento detectado
      Serial.println("Movimento detectado");
      // Serial.println(totalAcceleration);
    }
  } 

  
  // Verificar se um dedo está no sensor MAX30105
  bool fingerDetected = checkForFinger();

  if (fingerDetected) {
    uint32_t irBuffer[100];
    uint32_t redBuffer[100];
    int32_t bufferLength = 100;
    int32_t spo2;
    int8_t validSPO2;
    int32_t heartRate;
    int8_t validHeartRate;
    int limiteHRMinimo = 60;
    int limiteHRMaximo = 120;
    int limiteSpO2Minimo = 80;
    int limiteSpO2Maximo = 100;

    for (byte i = 0; i < bufferLength; i++) {
      while (particleSensor.available() == false)
        particleSensor.check();

      redBuffer[i] = particleSensor.getIR();
      irBuffer[i] = particleSensor.getRed();
      particleSensor.nextSample();
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Impressão de medidas// Impressão de medidas
    Serial.print("AccelX:"); Serial.print(a.acceleration.x); Serial.print(",");
    Serial.print("AccelY:"); Serial.print(a.acceleration.y); Serial.print(",");
    Serial.print("AccelZ:"); Serial.print(a.acceleration.z); Serial.print(", ");
    Serial.print("GyroX:"); Serial.print(g.gyro.x); Serial.print(",");
    Serial.print("GyroY:"); Serial.print(g.gyro.y); Serial.print(",");
    Serial.print("GyroZ:"); Serial.print(g.gyro.z); Serial.print("\n");

    if (totalAcceleration > thresholdTotalAcceleration) { 

      if (totalAcceleration > thresholdTotalAcceleration && verticalAcceleration > thresholdVerticalAcceleration) {
      // Queda detectada
        Serial.println("Queda detectada!");
        // Serial.println(totalAcceleration);
        postFallToServer();
      // Você pode tomar ações aqui, como enviar um aviso.
      } else {
      // Movimento detectado
        Serial.println("Movimento detectado");
        // Serial.println(totalAcceleration);
      }
    } else if (0 < totalAcceleration < thresholdTotalAcceleration) {
    // Não há movimento significativo
      Serial.println("Parado");
    }


    heartRateMeasurements[currentMeasurement] = heartRate;

    // Incrementar o índice da medida atual e circular no array
    currentMeasurement = (currentMeasurement + 1) % numMeasurements;

    // Aumentar o contador de medidas não nulas
    if (heartRate > 0) {
      measurementCount++;
    }

    // Calcular a média das medidas não nulas
    int sum = 0;
    for (int i = 0; i < numMeasurements; i++) {
      if (heartRateMeasurements[i] > 0) {
        sum += heartRateMeasurements[i];
      }
    }

    if (measurementCount > 0) {
      averageHeartRate = sum / measurementCount;
      Serial.print("Average BPM:");
      Serial.print(averageHeartRate);
      Serial.print(", ");
    } else {
      Serial.println("No valid BPM measurements yet.");
    }

      // if (heartRate >= limiteHRMinimo && heartRate <= limiteHRMaximo) {
      // Serial.print("BPM:"); Serial.print(heartRate, DEC); Serial.print(", ");
      // Serial.print("HRvalid:1 \n");

      // } else {
      //   Serial.print("BPM:"); Serial.print(0); Serial.print(", ");// Leitura não válida
      //   Serial.print("HRvalid:0 \n");
    
    

    if (spo2 >= limiteSpO2Minimo && spo2 <= limiteSpO2Maximo) {
      Serial.print("SPO2:"); Serial.print(spo2, DEC); Serial.print(", ");
      Serial.print("SPO2Valid:1 \n");
    
    } else {
      Serial.print("SPO2:"); Serial.print(0); Serial.print(", ");// Leitura não válida
      Serial.print("SPO2Valid:0 \n");
    
    }

    // Leitura da temperatura do sensor MAX30105
    float temperature = particleSensor.readTemperature();  // Leitura original do sensor
    float correcao = 11.3;  // Valor a ser subtraído para corrigir a leitura

    // Aplicar a correção
    temperature += correcao;
    Serial.print("temperature=");
    Serial.print(temperature, 2);
    Serial.print("°C");

    float temperatureF = particleSensor.readTemperatureF();
    temperatureF += correcao;

    Serial.print(" temperature=");
    Serial.print(temperatureF, 2);
    Serial.print("°F");

    Serial.println();
    Serial.print("\n");

    if(validHeartRate && validSPO2) {
      postDataToServer(temperature, averageHeartRate, spo2, a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);
    }
  }

  delay(1000);  // Ajuste o atraso conforme necessário para a sua aplicação
}

bool checkForFinger() {
  // Faça a leitura do sensor de IR (infrared)
  uint32_t irValue = particleSensor.getIR();

  // Defina um limite para considerar a presença de um dedo
  uint32_t fingerThreshold = 3000; // Ajuste esse valor conforme necessário

  // Verifique se o valor de IR está acima do limite
  if (irValue > fingerThreshold) {
    return true; // Um dedo está presente
  } else {
    return false; // Nenhum dedo está presente
  }
}