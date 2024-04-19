#include <Wire.h>
#include "MAX30100.h"

MAX30100 particleSensor;

#define SAMPLING_RATE          MAX30100_SAMPRATE_100HZ
#define IR_LED_CURRENT         MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT        MAX30100_LED_CURR_27_1MA
#define PULSE_WIDTH            MAX30100_SPC_PW_1600US_16BITS
#define HIGHRES_MODE           true

uint16_t irValue, redValue;
const int n = 5; // número de valores a serem considerados para a média
uint16_t irValues[n], redValues[n];
uint8_t i = 0;
float MredValue;
float MirValue;

void setup() {
  Serial.begin(115200);

  if (!particleSensor.begin()) {
    Serial.println("Não foi possível inicializar o sensor. Verifique as conexões!");
    while (1);
  }

  Serial.println("Sensor MAX30105 inicializado com sucesso!");

  particleSensor.setMode(MAX30100_MODE_SPO2_HR);
  particleSensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
  particleSensor.setLedsPulseWidth(PULSE_WIDTH);
  particleSensor.setSamplingRate(SAMPLING_RATE);
  particleSensor.setHighresModeEnabled(HIGHRES_MODE);
}

void loop() {
  static unsigned long lastMeasurementTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastMeasurementTime >= 100) { // Medir a cada 1 minuto
    float glucoseConcentration = medirGlicose();
    exibirResultados(glucoseConcentration, MredValue, MirValue);
    lastMeasurementTime = currentTime;
  }
}

float medirGlicose() {
  particleSensor.update();
  particleSensor.getRawValues(&irValue, &redValue);

  irValues[i] = irValue;
  redValues[i] = redValue;
  i = (i + 1) % n;

  uint32_t sumRed = 0, sumIR = 0;
  for (int i = 0; i < n; i++) {
    sumRed += redValues[i];
    sumIR += irValues[i];
  }

  MredValue = sumRed / n;
  MirValue = sumIR / n;

  float concentration = MredValue * 0.1;
  return concentration;
}

void exibirResultados(float concentration, float MredValue, float MirValue) {
  Serial.print("Concentração de Glicose: ");
  Serial.println(concentration);

  float outputValue = (MredValue * MredValue * 0.000000056) + (MirValue * 0.00025);
  Serial.print("Blood Sugar: ");
  Serial.println(outputValue);
}