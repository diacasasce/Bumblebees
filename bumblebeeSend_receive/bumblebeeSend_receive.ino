#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the receiver structure
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
struct_message recievedData;

const int IN_A0 = A0; // analog input
const int IN_D0 = 14; // digital input
int value_A0;
bool value_D0;
unsigned long lastTime = 0;  
unsigned long timerDelay = 1500;  // send readings timer

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP280 bme; // I2C

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&recievedData, incomingData, sizeof(recievedData));
  String receivedMAC = String (String(recievedData.mac[0],HEX)+":"+String(recievedData.mac[1],HEX)+":"+String(recievedData.mac[2],HEX)+":"+String(recievedData.mac[3],HEX)+":"+String(recievedData.mac[4],HEX)+":"+String(recievedData.mac[5],HEX));
  receivedMAC.toUpperCase();
  if (receivedMAC != String(WiFi.macAddress()) ){
    String sensordata = "{\"temperature\":\"";
    sensordata +=recievedData.Temperature;
    sensordata +="\",\"pressure\":\"";
    sensordata +=recievedData.Pressure;
    sensordata +="\",\"altitude\":\"";
    sensordata +=recievedData.Altitude;
    sensordata +="\",\"humidity\":\"";
    sensordata +=recievedData.LevHumidity;
    sensordata +="\",\"humidityThreshold\":\"";
    sensordata +=recievedData.ThreshHumidity;
    sensordata +="\",\"MAC\":\"";
    sensordata +=receivedMAC;
    sensordata +="\"}";
    Serial.println("REPLICATED - > ");
    Serial.println(sensordata);
    //esp_now_send(broadcastAddress, (uint8_t *) &recievedData, sizeof(recievedData));
  }
}

void readSensorData (){
  myData.Temperature = bme.readTemperature();
  myData.Pressure = bme.readPressure()/100.0F;
  myData.Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  myData.LevHumidity = analogRead(IN_A0);
  myData.ThreshHumidity = digitalRead(IN_D0);
  WiFi.macAddress(myData.mac);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_recv_cb(OnDataRecv);
  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  pinMode (IN_A0, INPUT);
  pinMode (IN_D0, INPUT);
}

void loop() {
  if ((millis() - lastTime) > timerDelay) {
    readSensorData();
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.println("sending my Data");
    lastTime = millis();
  }
}
