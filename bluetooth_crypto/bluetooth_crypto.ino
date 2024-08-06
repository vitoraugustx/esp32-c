#include <Wire.h>
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

const int BLUETOOTH_MAX_BUFFER_SIZE = 60 * 5; // Número de minutos

const int numMeasurements = 10; // Número de medidas para calcular a média
int heartRateMeasurements[numMeasurements]; // Array para armazenar as medidas
int currentMeasurement = 0; // Índice da medida atual
int measurementCount = 0;
int averageHeartRate;

const char* ssid       = "VITOR-NOTEBOOK";
const char* password   = "12345678";

char cleartext[512] = {0};
char ciphertext[1024];

const char* ntpServer = "br.pool.ntp.org";
const long  gmtOffset_sec = - 1 * 3600;
const int   daylightOffset_sec = -3600;

ESP32Time rtc(gmtOffset_sec);

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool alreadySentKey = false;

byte aes_key[16]; // 16-byte (128-bit) AES key

// General initialization vector (use your own IVs in production for full security!!!)
byte aes_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

class MyServerCallbacks: public BLEServerCallbacks {
public:
  BLECharacteristic* pCharacteristic;

  MyServerCallbacks(BLECharacteristic* characteristic) : pCharacteristic(characteristic) {}

  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    alreadySentKey = false;  // Reset the flag when a new device connects
    Serial.println("Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    alreadySentKey = false;
    pServer->getAdvertising()->start();
    Serial.println("Client disconnected");
  }

  // void sendByteOnce() {
  //   // Convert AES key byte array to hexadecimal string
  //   char hexKey[sizeof(aes_key) * 2 + 1];
  //   for (int i = 0; i < sizeof(aes_key); i++) {
  //     sprintf(hexKey + 2 * i, "%02X", aes_key[i]);
  //   }
  //   hexKey[sizeof(aes_key) * 2] = '\0'; // Null-terminate the string

  //   // Debugging: Print the AES key in hex before sending
  //   Serial.print("Sending AES key: ");
  //   Serial.println(hexKey);

  //   // Set the characteristic value to the hex key string
  //   pCharacteristic->setValue(hexKey);

  //   // Notify the connected client
  //   pCharacteristic->notify();
  //   Serial.println("Sent value via Bluetooth");

  //   // Set the flag to true to ensure this is only sent once
  //   alreadySentKey = true;
  // }
};

// Crie uma lista de objetos JSON
StaticJsonDocument<50> jsonDocument;
JsonArray jsonArray = jsonDocument.to<JsonArray>();

AESLib aesLib;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  aesLib.set_paddingmode(paddingMode::CMS);

  setUpBluetooth();
}

void loop() {
  if (deviceConnected && !alreadySentKey) {
    generateKey();
    sendKeyOnce();
  }

  gatherData();
  sendValuesFromListViaBluetooth();

  delay(1000);  
}

void generateKey(){
  // Generate a random AES key
  for (int i = 0; i < sizeof(aes_key); i++) {
    aes_key[i] = random(0, 256);
  }
}

void sendKeyOnce() {
  delay(5000);
  // Convert AES key byte array to hexadecimal string
  char hexKey[sizeof(aes_key) * 2 + 1];
  for (int i = 0; i < sizeof(aes_key); i++) {
    sprintf(hexKey + 2 * i, "%02X", aes_key[i]);
  }
  hexKey[sizeof(aes_key) * 2] = '\0'; // Null-terminate the string

  // Debugging: Print the AES key in hex before sending
  Serial.print("Sending AES key: ");
  Serial.println(hexKey);

  // Set the characteristic value to the hex key string
  pCharacteristic->setValue(hexKey);

  // Notify the connected client
  pCharacteristic->notify();
  Serial.println("Sent value via Bluetooth");

  // Set the flag to true to ensure this is only sent once
  alreadySentKey = true;
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

void sendValuesFromListViaBluetooth() {
  if (deviceConnected) {
    Serial.println("Sending values via Bluetooth:");
    for (JsonVariant item : jsonArray) {
      String requestBody;
      char jsonString[200];

      // Serialize the JSON object to a string
      serializeJson(item, jsonString);

      sprintf(cleartext, jsonString);

      Serial.print("Original string: ");
      Serial.println(cleartext);

      byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block is written, provide your own new copy...
      String encrypted_data = encrypt_impl(cleartext, enc_iv);
      sprintf(ciphertext, "%s", encrypted_data.c_str());
      Serial.print("Encrypted data: ");
      Serial.println(encrypted_data);

      byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block is written, provide your own new copy...
      String decrypted = decrypt_impl(ciphertext, dec_iv);
      Serial.print("Decrypted data: ");
      Serial.println(decrypted);

      // Set the characteristic value
      pCharacteristic->setValue(ciphertext);

      // Notify the client
      pCharacteristic->notify();
      Serial.print("Sent JSON via bluetooth: ");
      Serial.println(jsonString);

      jsonArray.remove(item.as<int>()); // Remova o objeto da lista após o envio bem sucedido
    }
  }
}

void gatherData() {
  if (jsonArray.size() >= BLUETOOTH_MAX_BUFFER_SIZE) {
    jsonArray.remove(0);
  }
  JsonObject obj = jsonArray.createNestedObject();
  obj["timestamp"] = getDateTime();
  obj["temperature"] = random(35, 37);
  obj["heart_rate"] = random(60, 120);
  obj["saturation"] = random(92, 100); // Valor aleatório como exemplo
  obj["physical_id"] = getId();
}

void setUpBluetooth() {
  // Create the BLE Device
  BLEDevice::init("ESP32");
  BLEDevice::setMTU(517);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  MyServerCallbacks* callbacks = new MyServerCallbacks(pCharacteristic);
  pServer->setCallbacks(callbacks);

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
  char buffer[30];
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
