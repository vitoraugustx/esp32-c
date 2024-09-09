// Bibliotecas dos sensores e comunicação sensorial
#include <MAX3010x.h>
#include "filters.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// Bibliotecas para comunicação BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <ESP32Time.h>
#include <ArduinoJson.h>

// Biblioteca para armazenamento de dados na EEPROM
#include "EEPROM.h"

// Definições de identificação única do Bluetooth
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Definição do tamanho do Buffer de envio de dados Bluetooth
const int BLUETOOTH_MAX_BUFFER_SIZE = 60 * 5 /* Número de minutos*/;

// Definição do endereço de memória EEPROM
#define EEPROM_SIZE 64

int gmtOffset_sec = -10800; 
ESP32Time rtc(gmtOffset_sec);  // Assuming gmtOffset_sec is defined somewhere
StaticJsonDocument<50> jsonDocument;
JsonArray jsonArray = jsonDocument.to<JsonArray>();

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int txValue = 0;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
    Serial.println("Client disconnected");
  };
}; 

Adafruit_MPU6050 mpu;

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 1.0;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

int pressao_sistolica, pressao_diastolica;

float bpm_values[10];
float spo2_values[10];
int measure_counter = 0;

float update_predicted_glucose(float bpm, float spo2);

// Limiares para detecção de movimento e queda
float thresholdTotalAcceleration = 10.0; // Ajuste esse valor conforme necessário
float thresholdVerticalAcceleration = 13; // Ajuste esse valor conforme necessário

// Variáveis para armazenar o nome do dispositivo Bluetooth e variáveis de comunicação Serial
String bluetoothName = "";  // Global string to store the received data
bool serialDataReceived = false;

void setup() {
  Serial.begin(115200);

  // Inicialização da EEPROM
  initEPROM();
  // clearEEPROM();

  // Leitura do nome do dispositivo Bluetooth da EEPROM
  bluetoothName = readEEPROM();
  if (bluetoothName == "null"){
    // Se o nome do dispositivo não foi definido, solicita o nome do dispositivo via Serial
    waitForBluetoothNameFromSerial();
  }

  setUpBluetooth();
  // Inicialização do MPU6050
  if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
  } else {
      mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Inicialização do sensor MAX30105
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor initialized");
    Serial.println(getId());
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
 
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;


void gatherData(float temp, int16_t heartRate, int16_t spo2, int bpSys = -1, int bpDia = -1, float predictedGlucose = -1) {
    if(jsonArray.size() >= BLUETOOTH_MAX_BUFFER_SIZE) {
      jsonArray.remove(0);
    }
    JsonObject obj = jsonArray.createNestedObject();
    obj["temperature"] = temp;
    obj["heartRate"] = heartRate;
    obj["saturation"] = spo2;
    obj["sysPressure"] = bpSys;
    obj["diaPressure"] = bpDia;
    obj["physical_id"] = getId();
    obj["predictedGlucose"] = predictedGlucose;
    
}

void gatherDataMPU(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
    if(jsonArray.size() >= BLUETOOTH_MAX_BUFFER_SIZE) {
      jsonArray.remove(0);
    }
    JsonObject obj = jsonArray.createNestedObject();
    obj["Acceleration X"] = accX;
    obj["Acceleration Y"] = accY;
    obj["Acceleration Z"] = accZ;
    obj["Gyro X"] = gyroX;
    obj["Gyro Y"] = gyroY;
    obj["Gyro Z"] = gyroZ;
    obj["physical_id"] = getId();
}

void sendValuesFromListViaBluetooth() {
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

void loop() {
  // Leitura e cálculo da aceleração usando MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  

  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
  // Detect Finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if(!isnan(current_diff) && !isnan(last_diff)) {
      
      // Detect Heartbeat - Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detect Heartbeat - Falling Edge Threshold
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          int bpm = 60000/(crossed_time - last_heartbeat);
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          float r = rred/rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;


          Serial.print("bpm: ");
          Serial.print(bpm);
          Serial.print("  spo2: ");
          Serial.println(spo2);

          
          if (bpm > 50 && bpm < 200 && spo2 > 85 && spo2 <= 100) {
            // Armazenar valores de bpm e spo2 nas últimas 10 medidas
            bpm_values[measure_counter % 10] = bpm;
            spo2_values[measure_counter % 10] = spo2;
            measure_counter++;

            Serial.print("PIF bpm: ");
            Serial.print(bpm);
            Serial.print("  PIF spo2: ");
            Serial.println(spo2);

            //float temperatura = tempCelsius();
            //gatherData(temperatura, bpm, spo2);

            // Se já foram feitas 10 medidas, calcular predicted_glucose
            if (measure_counter >= 10) {
              int size = 10;

              // Substituir outliers pela média
              replace_outliers_with_mean(bpm_values, size);
              replace_outliers_with_mean(spo2_values, size);

              float sum_bpm = 0;
              float sum_spo2 = 0;
              for (int i = 0; i < 10; i++) {
                  sum_bpm += bpm_values[i];
                  sum_spo2 += spo2_values[i];
              }
              float avg_bpm = sum_bpm / 10;
              float avg_spo2 = sum_spo2 / 10;

              Serial.print("avg_bpm: ");
              Serial.println(avg_bpm);
              Serial.print("avg_spo2: ");
              Serial.println(avg_spo2);

              float predicted_glucose = update_predicted_glucose(avg_bpm, avg_spo2);
              // Printar o valor da glicose predita
              Serial.print("Predicted glucose: ");
              Serial.println(predicted_glucose);

              // Printar o valor da temperatura
              Serial.print("Temperatura: ");
              Serial.println(tempCelsius());
              float temperatura = tempCelsius();

              // Call your function to calculate blood pressure
              calcularPressaoArterial(avg_bpm, avg_spo2, &pressao_sistolica, &pressao_diastolica);

              // Printar o valor da pressão arterial
              Serial.print("Pressão sistolica: ");
              Serial.println(pressao_sistolica);
              Serial.print("Pressão diastolica: ");
              Serial.println(pressao_diastolica);
                
                // Limpa o contador e envia os dados, incluindo a glicose prevista
              gatherData(temperatura, avg_bpm, avg_spo2, pressao_sistolica, pressao_diastolica, predicted_glucose);
                
              sendValuesFromListViaBluetooth();

              // Reiniciar contagem das medidas
              measure_counter = 0;
            } else {

              Serial.print("bpm Livre: ");
              Serial.print(bpm);
              Serial.print("Spo2 Livre: ");
              Serial.println(spo2);

              float temperatura = tempCelsius();
              //gatherData(temperatura, bpm, spo2);
              gatherDataMPU(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);
                
                // Continua acumulando dados sem a glicose
                //gatherData(temperatura, bpm, spo2, pressao_sistolica, pressao_diastolica);
                
              sendValuesFromListViaBluetooth();
            }
          } 

          
          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }

        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
}

float update_predicted_glucose(float bpm, float spo2) {
    // Calcular a glicose predita
    float new_predicted_glucose = -494.23133217811767 
                                + (7.686066281779009 * bpm) 
                                + (5.668788197282953 * spo2) 
                                + (0.006448777577223952 * pow(bpm, 2)) 
                                + (-0.09730249584572759 * bpm * spo2) 
                                + (0.015323542439047877 * pow(spo2, 2));

    return new_predicted_glucose;
}

// Leitura da temperatura do sensor MAX30105
float tempCelsius(){

  float temperature = sensor.readTemperature();  // Leitura original do sensor
  float correcao = 0;  // Valor a ser subtraído para corrigir a leitura

  // Aplicar a correção
  temperature += correcao;
  return temperature;
}


// Função para calcular a pressão arterial
void calcularPressaoArterial(int16_t heartRate, int16_t SpO2, int *pressao_sistolica, int *pressao_diastolica) {

  // Algoritmo de regressão linear para estimar a pressão arterial sistólica e diastólica

  // Coeficientes da regressão linear (baseados em dados empíricos)
  float coef_a = -25.147;
  float coef_b = 1.11;
  float coef_c = 0.613;

  // Calcula a pressão arterial sistólica
  *pressao_sistolica = (int)(coef_a + coef_b * SpO2 + coef_c * heartRate);

  // Estima a pressão arterial diastólica como 2/3 da pressão arterial sistólica
  *pressao_diastolica = (int)(*pressao_sistolica*2/3);
}

void waitForBluetoothNameFromSerial(){
  unsigned long lastSendTime = millis();
  int sendInterval = 1000; // Intervalo de envio de dados

  // Loop para aguardar o nome do dispositivo via Serial
  while (!serialDataReceived) {
    // Chama a função para obter o nome do dispositivo via Serial
    getBluetoothName();

    
    if (millis() - lastSendTime > sendInterval) {
      Serial.println(getId()); // Envia o ID do dispositivo via Serial
      lastSendTime = millis(); 
    }
  }
}

void getBluetoothName(){
  if(Serial.available() > 0){
    bluetoothName = Serial.readStringUntil('\n'); // Lê os dados até o caractere de nova linha
    serialDataReceived = true;
    writeEEPROM(bluetoothName);
  }
}

void initEPROM(){
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("failed to init EEPROM");
    while(1);
  }
}

void writeEEPROM(String data) {
  clearEEPROM(); // Optional: clear EEPROM before writing new data
  int length = data.length();

  for (int i = 0; i < EEPROM_SIZE && i < length; i++) {
    EEPROM.write(i, data[i]);
  }
  EEPROM.commit();
}

String readEEPROM() {
  String result = "";
  bool foundChar = false;

  for (int i = 0; i < EEPROM_SIZE; i++) {
    byte readValue = EEPROM.read(i);

    if (readValue != 0) {
      foundChar = true;
      result += char(readValue);
    }
  }

  if (!foundChar) {
    return "null";
  }

  return result;
}

void clearEEPROM() {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
}

void setUpBluetooth(){
  // Create the BLE Device
  BLEDevice :: init(bluetoothName);
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

void replace_outliers_with_mean(float list[], int size) {
    int i;
    double sum = 0, mean, variance = 0, stddev;

    // Calcular a média
    for (i = 0; i < size; i++) {
        sum += list[i];
    }
    mean = sum / size;

    // Calcular a variância
    for (i = 0; i < size; i++) {
        variance += pow(list[i] - mean, 2);
    }
    variance /= size;
    stddev = sqrt(variance);

    // Calcular os limites para outliers (2 desvios padrão da média)
    double lower_bound = mean - 2 * stddev;
    double upper_bound = mean + 2 * stddev;

    // Substituir outliers pela média
    for (i = 0; i < size; i++) {
        if (list[i] < lower_bound || list[i] > upper_bound) {
            list[i] = (float)mean;
        }
    }
}
