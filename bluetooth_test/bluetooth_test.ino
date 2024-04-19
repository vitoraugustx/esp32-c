#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include <WiFi.h>
#include "time.h"
#include <ESP32Time.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

const int BLUETOOTH_MAX_BUFFER_SIZE = 60 * 5 /* Número de minutos*/;

const int numMeasurements = 10; // Número de medidas para calcular a média
int heartRateMeasurements[numMeasurements]; // Array para armazenar as medidas
int currentMeasurement = 0; // Índice da medida atual
int measurementCount = 0;
int averageHeartRate;

const char* ssid       = "VITOR-NOTEBOOK";
const char* password   = "12345678";

const char* ntpServer = "br.pool.ntp.org";
const long  gmtOffset_sec = - 1 * 3600;
const int   daylightOffset_sec = -3600;

ESP32Time rtc(gmtOffset_sec);

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int txValue = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->getAdvertising()->start();
    }
}; 



// Crie uma lista de objetos JSON
StaticJsonDocument<50> jsonDocument;
JsonArray jsonArray = jsonDocument.to<JsonArray>();


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  connectToWifi();

  /*---------set with NTP---------------*/
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)){
    rtc.setTimeStruct(timeinfo); 
  }

  setUpBluetooth();
  
  // TODO: descomentar
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  // mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // TODO: descomentar
  /*if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
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
  particleSensor.enableDIETEMPRDY(); // Enable the temp ready interrupt. This is required.*/
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
  
  
  // TODO: descomentar
  /*
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
  */ 

  // TODO: descomentar
  /*
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
    Serial.print("AccelZ:"); Serial.print(a.acceleration.z); Serial.print(",");
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

  */


  gatherData();
  sendValuesFromListViaBluetooth();

  // sendValueViaBluetooth(32.5, 87, 97, 3.5, 0.5, 4.6, 0.0, 1.2, 3.3);

  delay(1000);  // Ajuste o atraso conforme necessário para a sua aplicação
}

void sendValuesFromListViaBluetooth(){
  if (deviceConnected) {
    Serial.println("Sending values via Bluetooth:");
    for (JsonVariant item : jsonArray) {
      String requestBody;
      char jsonString[200];
      // Serial.print(item);

      // Serialize the JSON object to a string
      serializeJson(item, jsonString);
      // serializeJson(doc, requestBody);

      // Set the characteristic value
      pCharacteristic->setValue(jsonString);

      // Notify the client
      pCharacteristic->notify();
      Serial.print("Sent JSON via bluetooth: ");
      Serial.println(jsonString);

      jsonArray.remove(item.as<int>()); // Remova o objeto da lista após o envio bem sucedido

    }
  }
}

void gatherData(){
  if(jsonArray.size() >= BLUETOOTH_MAX_BUFFER_SIZE){
    jsonArray.remove(0);
  }
  JsonObject obj = jsonArray.createNestedObject();
  obj["timestamp"] = getDateTime();
  obj["temperature"] = random(35, 37);
  obj["heart_rate"] = random(60, 120);
  obj["saturation"] = random(92, 100); // Valor aleatório como exemplo
  obj["physical_id"] = getId();
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

//connect to WiFi
void connectToWifi(){
  // Serial.printf("Connecting to %s ", ssid);
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println(" CONNECTED");
  wifiSetup();
  wifiConnect();
}

void wifiSetup() {
  WiFi.onEvent(onWiFiEvent);
}

void wifiConnect() {
  WiFi.begin(ssid, password);
}

void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi begin failed ");

      // try reconnect here (after delay???)
      delay(60 * 1000);
      wifiConnect();
      break;

    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi begin succeeded ");
      // Connected successfully
      break;      
  }
}

void setUpBluetooth(){
  // Create the BLE Device
  BLEDevice :: init("ESP32");
  BLEDevice :: setMTU(517);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    // BLECharacteristic::PROPERTY_READ |
    // BLECharacteristic::PROPERTY_WRITE | 
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // BLE2902 needed to notify
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

String getDateTime() {
  struct tm timeinfo = rtc.getTimeStruct();
  // if(!getLocalTime(&timeinfo)){
  //   Serial.println("Failed to obtain time");
  //   return "null";
  // }
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
    
  http.begin("https://atech-main-api-dev.onrender.com/api/warnings/fall");  
  // http.begin("http://192.168.8.102:3000/data");
  http.addHeader("Content-Type", "application/json");         
  
  StaticJsonDocument<200> doc;

  JsonObject obj = doc.createNestedObject();

  // Serial.printf("\nCHIP MAC: %012llx\n", ESP.getEfuseMac());
  
  // create an object
  JsonObject object = doc.to<JsonObject>();
  object["physical_id"] = getId();
  object["timestamp"] = getDateTime();
  object["type"] = "Fall";
    
  String requestBody;
  serializeJson(doc, requestBody);
  Serial.println("JSON: " + requestBody);
    
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
    
  http.begin("https://atech-main-api-dev.onrender.com/api/wearables/insertData");  
  // http.begin("http://192.168.8.102:3000/data");
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
  object["saturation"] = spo2;
    
  String requestBody;
  serializeJson(doc, requestBody);

  Serial.println("JSON: " + requestBody);
    
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

void sendValueViaBluetooth(float temperature, int32_t heartRate, int32_t spo2, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ){
  if (deviceConnected) {

    // Create a JSON object

    StaticJsonDocument<200> doc;

    // Create a JsonArray to hold the documents
    StaticJsonDocument<400> jsonArray;

    JsonObject obj = doc.createNestedObject();

    // Serial.printf("\nCHIP MAC: %012llx\n", ESP.getEfuseMac());
    
    // create an object
    JsonObject object = doc.to<JsonObject>();
    object["physical_id"] = getId();
    object["timestamp"] = getDateTime();
    object["temperature"] = temperature;
    object["heartRate"] = heartRate;
    object["saturation"] = spo2;

    // Add the documents to the array
    jsonArray.add(doc);
      
    String requestBody;
    
    // Serialize the JSON object to a string
    char jsonString[200];
    serializeJson(jsonArray, jsonString);
    // serializeJson(doc, requestBody);

    // Set the characteristic value
    pCharacteristic->setValue(jsonString);

    // Notify the client
    pCharacteristic->notify();
    Serial.print("Sent JSON via bluetooth: ");
    Serial.println(jsonString);

  }
}