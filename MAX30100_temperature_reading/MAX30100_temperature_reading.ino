#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000

// Create a MAX30100 object
MAX30100 sensor;

// Time when the last reading was taken
uint32_t tsLastReading = 0;

void setup() {
  Serial.begin(115200);

  Serial.print("Initializing MAX30100..");

  // Initialize sensor
  if (!sensor.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }

  sensor.setMode(MAX30100_MODE_SPO2_HR);
  sensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_27_1MA);
  sensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
  sensor.setSamplingRate(MAX30100_SAMPRATE_100HZ);
  sensor.setHighresModeEnabled(true);
}

void loop() {
  sensor.update();

  if (millis() - tsLastReading > REPORTING_PERIOD_MS) {
    sensor.startTemperatureSampling();
    if (sensor.isTemperatureReady()) {
      float temp = sensor.retrieveTemperature();
      Serial.print("Temperature = ");
      Serial.print(temp);
      Serial.print("*C | ");
      Serial.print((temp * 9.0) / 5.0 + 32.0);
      Serial.println("*F");
    }
    tsLastReading = millis();
  }
}