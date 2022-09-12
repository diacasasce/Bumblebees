#include <ESP8266WiFi.h>
#include <espnow.h>

// Structure example to receive data
typedef struct struct_message {
  float Temperature;
  float Pressure;
  float Altitude;
  int LevHumidity;
  byte ThreshHumidity;
  byte mac[6];   
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  String sensordata = "{\"temperature\":\"";
  sensordata +=myData.Temperature;
  sensordata +="\",\"pressure\":\"";
  sensordata +=myData.Pressure;
  sensordata +="\",\"altitude\":\"";
  sensordata +=myData.Altitude;
  sensordata +="\",\"humidity\":\"";
  sensordata +=myData.LevHumidity;
  sensordata +="\",\"humidityThreshold\":\"";
  sensordata +=myData.ThreshHumidity;
  sensordata +="\",\"MAC\":\"";
  sensordata +=String(myData.mac[0],HEX);
  sensordata +=":";
  sensordata +=String(myData.mac[1],HEX);
  sensordata +=":";
  sensordata +=String(myData.mac[2],HEX);
  sensordata +=":";
  sensordata +=String(myData.mac[3],HEX);
  sensordata +=":";
  sensordata +=String(myData.mac[4],HEX);
  sensordata +=":";
  sensordata +=String(myData.mac[5],HEX);
  sensordata +="\"}";

  Serial.println(sensordata);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  
}
