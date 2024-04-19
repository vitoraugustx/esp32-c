#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000
#define LED_BUILTIN 2

int lastOccurrence = LOW;
bool firstDelay = true;

// Create a MAX30100 object
MAX30100 sensor;

uint16_t ir, red;
uint16_t avg_ir = 0, avg_red = 0;
int SAMPLE_NUMBER = 5000;
int i = 0;

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Initializing MAX30100..");

    // Initialize sensor
    if (!sensor.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

    configureMax30100();

    takeSampleReadings();
}

void loop() {
  sensor.update();
  
  while (sensor.getRawValues(&ir, &red) && i <= SAMPLE_NUMBER) {
    if (ir > 10*avg_ir && red > 10*avg_red) {
      Serial.print(ir);
      Serial.print(',');
      Serial.println(red);
      i += 1;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void configureMax30100() {
  sensor.setMode(MAX30100_MODE_SPO2_HR);
  sensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_27_1MA);
  sensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
  sensor.setSamplingRate(MAX30100_SAMPRATE_100HZ);
  sensor.setHighresModeEnabled(true);
}

void takeSampleReadings() {
  delay(50);
  for (int i = 0; i <= 9; i++) {
    sensor.update();
    sensor.getRawValues(&ir, &red);
    avg_ir += ir;
    avg_red += red;
    delay(50);
  }
  avg_ir /= 10;
  avg_red /= 10;
}