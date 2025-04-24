#include <WiFi.h>
#include <esp_now.h>

// Define the struct as received from the slave
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

void onDataReceive(const uint8_t *macAddr, const uint8_t *data, int len) {
  // Ensure the incoming data is of the correct size
  if (len == sizeof(SensorData)) {
    memcpy(&receivedData, data, sizeof(SensorData));
    Serial.println("Received SensorData via ESP-NOW");

    // Print the received data
    Serial.print("Timestamp: ");
    Serial.println(receivedData.timestamp);
    Serial.print("AccelX: ");
    Serial.println(receivedData.accelX);
    // Print additional data as needed...
    Serial.print("Latitude: ");
    Serial.println(receivedData.latitude);
    Serial.print("Longitude: ");
    Serial.println(receivedData.longitude);
    Serial.print("Satellites: ");
    Serial.println(receivedData.satellites);
    Serial.println();
  } else {
    Serial.println("Received data size mismatch.");
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
  Serial.println("Receiver MAC Address: " + WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    while (true); // Halt if failed
  }

  // Set the callback function to handle incoming data
  esp_now_register_recv_cb(onDataReceive);

  Serial.println("Receiver (Ground Station) Ready");
}

void loop() {
  // Main loop remains empty since ESP-NOW is interrupt-driven
  // All work is done in the onDataReceive callback
}
