const int ACTIVITY_LED_PIN = 2;  // Define the pin for activity LED

void setup() {
  Serial.begin(115200);  // Start serial communication
  pinMode(ACTIVITY_LED_PIN, OUTPUT); // Set activity LED pin as output
}

void loop() {
  // Check if the serial port is connected
  if (Serial) {
    // Serial port is connected (implies USB is active)
    Serial.println("Serial port connected");
    
    digitalWrite(ACTIVITY_LED_PIN, 1);
  
  } else {
    digitalWrite(ACTIVITY_LED_PIN, 0);

  }
  delay(1000);
}
