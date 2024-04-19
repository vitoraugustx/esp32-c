/* Documentação da rotina de testes:
- A rotina de testes é responsável por verificar se os sensores MAX30100 e MPU6050 estão funcionando corretamente. Está sendo utilizado o led interno (led azul - pino 2) do ESP32 para indicar o status de ambos os sensores.
- 2 piscadas: falha no sensor MAX30100
- 1 piscada: falha no sensor MPU6050
 */


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000
#define INTERNAL_LED 2

Adafruit_MPU6050 mpu;
PulseOximeter pox;

void setup() {
  Serial.begin(115200);
  pinMode(INTERNAL_LED, OUTPUT);
  
}

void loop() {
  if(initializeMPU6050()){
    Serial.println("MPU6050 initialized");
  } else {
    Serial.println("MPU6050 not initialized");
    ledBlinkOnce();
  }
  delay(500);
  if(initializeMAX30100()){
    Serial.println("MAX30100 initialized");
  } else {
    Serial.println("MAX30100 not initialized");
    ledBlinkTwice();
  }
  
  delay(1000);

}

bool initializeMPU6050(){
  // Try to initialize!
  if (!mpu.begin()) {
    return false;
  }
  return true;
}

bool initializeMAX30100(){
  if (!pox.begin()) {
    return false;
  } 
  return true;
  
}

void ledBlinkOnce(){
  digitalWrite (INTERNAL_LED, HIGH); 
  delay (500); 
  digitalWrite (INTERNAL_LED, LOW); 
  delay (500); 
}

void ledBlinkTwice(){
  digitalWrite (INTERNAL_LED, HIGH); 
  delay (100); 
  digitalWrite (INTERNAL_LED, LOW); 
  delay (100); 
  digitalWrite (INTERNAL_LED, HIGH); 
  delay (100); 
  digitalWrite (INTERNAL_LED, LOW); 
  delay (100); 
}
