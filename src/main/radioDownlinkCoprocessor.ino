#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

#define I2C_SLAVE_ADDR 0x42

struct SensorData {
   unsigned long timestamp;
    float accelX, accelY, accelZ;  // Acceleration (m/s^2)
    float rawAX, rawAY, rawAZ; // raw values for debugging
    float gyroX, gyroY, gyroZ;    // Angular velocity (deg/s)
    float roll, pitch, yaw; // Roll, pitch, yaw (degrees)
    float velocityX, velocityY, velocityZ; // Velocity (m/s)
    float positionX, positionY, positionZ; // Position (m)
    float altitude; // Altitude (m)
    bool liftoffDetected; // Flag for liftoff detection
    unsigned long liftoffTime; // Timestamp of liftoff
    bool apogeeDetected; // Flag for apogee detection
    unsigned long apogeeTime; // Timestamp of apogee
    float rateOfChange;  //(m/s)
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
