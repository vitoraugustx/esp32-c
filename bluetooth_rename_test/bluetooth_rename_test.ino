// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include "EEPROM.h"
// #include "MAX30105.h"
// #include "spo2_algorithm.h"
// #include "heartRate.h"

#include <WiFi.h>
#include "time.h"
#include <ESP32Time.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#include <AESLib.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

int addr = 0;
#define EEPROM_SIZE 64

// Adafruit_MPU6050 mpu;
// MAX30105 particleSensor;

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

int ledPin = 2;
String bluetoothName = "";  // Global string to store the received data
bool dataReceived = false;

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool alreadySentKey = false;

byte aes_key[] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66 }; // 16-byte (128-bit) AES key

// General initialization vector (use your own IVs in production for full security!!!)
byte aes_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    alreadySentKey = false;  // Reset the flag when a new device connects
    Serial.println("Client connected");
    sendByteOnce();  // Send the byte array immediately upon connection
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
    Serial.println("Client disconnected");
  }

  void sendByteOnce() {
    if (deviceConnected && !alreadySentKey) {     

      // Set the characteristic value to the byte array
      pCharacteristic->setValue(aes_key, sizeof(aes_key));

      // Notify the connected client
      pCharacteristic->notify();
      Serial.println("Sent value via Bluetooth");

      // Set the flag to true to ensure this is only sent once
      alreadySentKey = true;
    }
  }
}; 

// Crie uma lista de objetos JSON
StaticJsonDocument<50> jsonDocument;
JsonArray jsonArray = jsonDocument.to<JsonArray>();

AESLib aesLib;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  initEPROM();

  // clearEEPROM();

  bluetoothName = readEEPROM();

  if (bluetoothName == "null"){
    waitForBluetoothNameFromSerial();
  }  

  setUpBluetooth();

}

void loop() {

  gatherData();
  // sendValuesFromListViaBluetooth();

  // sendValueViaBluetooth(32.5, 87, 97, 3.5, 0.5, 4.6, 0.0, 1.2, 3.3);

  delay(1000);  
}

void initEPROM(){
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("failed to init EEPROM");
    while(1);
  }
}

void clearEEPROM() {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
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

void writeEEPROM(String data) {
  clearEEPROM(); // Optional: clear EEPROM before writing new data
  int length = data.length();

  for (int i = 0; i < EEPROM_SIZE && i < length; i++) {
    EEPROM.write(i, data[i]);
  }
  EEPROM.commit();
}

void getBluetoothName(){
  if(Serial.available() > 0){
    bluetoothName = Serial.readStringUntil('\n'); // Read the data until newline
    dataReceived = true;
    writeEEPROM(bluetoothName);
  }
}

void waitForBluetoothNameFromSerial(){

  // Initialize timing for sending data periodically
  unsigned long lastSendTime = millis();
  int sendInterval = 1000; // Interval at which to send data (1 second)

  // Loop until data is completely received
  while (!dataReceived) {
    // Call function to handle incoming data
    getBluetoothName();

    // Check if it's time to send data again
    if (millis() - lastSendTime > sendInterval) {
      Serial.println(getId()); // Simulate sending data
      lastSendTime = millis(); // Reset the timer
    }
  }
}

String encrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen] = {0};
  aesLib.encrypt64((const byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}

String decrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = {0}; // Half may be enough
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
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

      String encrypted = encrypt_impl(jsonString, aes_iv);

      // Set the characteristic value
      pCharacteristic->setValue(encrypted);

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