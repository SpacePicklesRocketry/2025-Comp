#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

#define I2C_SLAVE_ADDR 0x42

struct SensorData {
  unsigned long timestamp;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float roll, pitch, yaw;
  float velocityX, velocityY, velocityZ;
  float positionX, positionY, positionZ;
  float altitude;
  bool liftoffDetected;
  unsigned long liftoffTime;
  bool apogeeDetected;
  unsigned long apogeeTime;
  float rateOfChange;
  float latitude;
  float longitude;
  int satellites;
};

SensorData receivedData;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onReceive(int numBytes) {
  if (numBytes == sizeof(SensorData)) {
    Wire.readBytes((char *)&receivedData, sizeof(SensorData));
    Serial.println("Received SensorData over I²C.");

    // Send over ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&receivedData, sizeof(SensorData));
    Serial.println("SensorData broadcast via ESP-NOW.");
  } else {
    Serial.print("Unexpected data size over I²C: ");
    Serial.println(numBytes);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("MAC: " + WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true);
  }

  // Add peer (broadcast)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_add_peer(&peerInfo);
  }

  // Init I²C
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onReceive);

  Serial.println("I²C Slave + ESP-NOW Broadcaster Ready");
}

void loop() {
  // Passiveloop
}
