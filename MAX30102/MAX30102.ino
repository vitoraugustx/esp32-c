#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;

uint32_t irBuffer[100];
uint32_t redBuffer[100];

#define REPORTING_PERIOD_MS 1000

void setup()
{
  Serial.begin(115200);
  
  Serial.print("Initializing Pulse Oximeter..");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 50;
  byte sampleAverage = 1;
  byte ledMode = 2;
  byte sampleRate = 100;
  int pulseWidth = 69;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void loop()
{
  int32_t bufferLength = 100;
  int32_t spo2;
  int8_t validSPO2;
  int32_t heartRate;
  int8_t validHeartRate;

  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[i] = particleSensor.getIR();
    irBuffer[i] = particleSensor.getRed();
    particleSensor.nextSample();

    //Serial.print(F("red: "));
    //Serial.print(redBuffer[i], DEC);
    //Serial.print(F("\t ir: "));
    //Serial.println(irBuffer[i], DEC);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  Serial.print(F("HR: "));
  Serial.print(heartRate, DEC);
  Serial.print(F("\tHRvalid: "));
  Serial.print(validHeartRate, DEC);
  Serial.print(F("\tSPO2: "));
  Serial.print(spo2, DEC);
  Serial.print(F("\tSPO2Valid: "));
  Serial.println(validSPO2, DEC);

  delay(REPORTING_PERIOD_MS);
}