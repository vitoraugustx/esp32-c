#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int txValue = 0;

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
}; 

void setup(){
Serial.begin(115200);

// Create the BLE Device
BLEDevice :: init("ESP32");

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

void loop(){
  if (deviceConnected) {
    txValue = txValue + 1;

    // Conversin of txValue
    char txString[8];
    dtostrf(txValue, 1, 2, txString);

    // Set the characteristic value
    pCharacteristic->setValue(txString);

    // Notify the client
    pCharacteristic->notify();
    Serial.print("Sent value: ");
    Serial.println(txString);
    delay(1000);
  }
}