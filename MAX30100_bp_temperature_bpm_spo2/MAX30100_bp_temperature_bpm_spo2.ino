/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000
#define PRINT_INTERVAL_MS 1000
unsigned long lastPrintTime = 0;  // Variable to store the time of the last print


// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
PulseOximeter pox;

// Create a MAX30100 object
MAX30100 sensor;

uint32_t tsLastReport = 0;

int16_t heartRate;
int16_t SpO2;
int pressao_sistolica, pressao_diastolica;

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    // Serial.println("Beat!");
}

void setup()
{
    Serial.begin(115200);

    Serial.print("Initializing pulse oximeter..");

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

    // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check MAX30100_Registers.h for all the
    //   available options.
    // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    sensor.setMode(MAX30100_MODE_SPO2_HR);
    sensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_27_1MA);
    sensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
    sensor.setSamplingRate(MAX30100_SAMPRATE_100HZ);
    sensor.setHighresModeEnabled(true);

    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
    // Make sure to call update as fast as possible
    pox.update();

    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        
      heartRate = pox.getHeartRate();
      SpO2 = pox.getSpO2();

      // Realiza o cálculo da pressão arterial
      // int pressao_sistolica, pressao_diastolica;

      float temp;
      sensor.startTemperatureSampling();
      if (sensor.isTemperatureReady()) {
        float aux = sensor.retrieveTemperature();
        if (aux <= 40){
          temp = aux;
        }
        Serial.print("Temperature = ");
        Serial.print(temp);
        Serial.print("*C | ");
        Serial.print((temp * 9.0) / 5.0 + 32.0);
        Serial.println("*F");
      }
      tsLastReport = millis();
    }

    // Check if it's time to print again
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();  // Update the last print time

        // Call your function to calculate blood pressure
        calcularPressaoArterial(heartRate, SpO2, &pressao_sistolica, &pressao_diastolica);

        // Print the results
        Serial.print("Frequência cardíaca: ");
        Serial.println(heartRate);
        Serial.print("SpO2: ");
        Serial.println(SpO2);
        Serial.print("Pressão sistólica: ");
        Serial.println(pressao_sistolica);
        Serial.print("Pressão diastólica: ");
        Serial.println(pressao_diastolica);

        Serial.println("---------------------------------");
    }
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
  *pressao_diastolica = (int)(*pressao_sistolica * 2 / 3);
}
