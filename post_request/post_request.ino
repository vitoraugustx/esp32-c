#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>
 
const char *AP_SSID = "VETORIAL_20";
const char *AP_PWD = "mavi2001";
  
WiFiMulti wifiMulti;
 
void setup() {
  Serial.begin(9600);
   
  delay(4000);
  wifiMulti.addAP(AP_SSID, AP_PWD);
 
  postDataToServer();
}
 
void loop() {
  // Not used in this example
}

String obterEnderecoMAC() {
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
 
void postDataToServer() {

  char id_buffer[250];

  uint32_t low     = ESP.getEfuseMac() & 0xFFFFFFFF; 
  uint32_t high    = ( ESP.getEfuseMac() >> 32 ) % 0xFFFFFFFF;
  uint64_t fullMAC =  ESP.getEfuseMac();
 
  Serial.println("Posting JSON data to server...");
  // Block until we are able to connect to the WiFi access point
  if (wifiMulti.run() == WL_CONNECTED) {
     
    HTTPClient http;   
     
    http.begin("http://192.168.2.103:3000/data");  
    http.addHeader("Content-Type", "application/json");         
     
    StaticJsonDocument<200> doc;

    JsonObject obj = doc.createNestedObject();

    Serial.printf("\nCHIP MAC: %012llx\n", ESP.getEfuseMac());
   
    obj["physical_id"] = obterEnderecoMAC();
    obj["time"] = 1351824120;
    obj["heartRate"] = 64;
    obj["saturation"] = 99;
     
    String requestBody;
    serializeJson(doc, requestBody);
     
    int httpResponseCode = http.POST(requestBody);
 
    if(httpResponseCode>0){
       
      String response = http.getString();                       
       
      Serial.println(httpResponseCode);   
      Serial.println(response);
     
    }
    // else {
     
    //   Serial.printf("Error occurred while sending HTTP POST: %s\n", httpClient.errorToString(statusCode).c_str());
       
    // }
     
  }
}