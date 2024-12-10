#include <Wire.h>
#include "Sensors.h"
#include "KalmanFilter.h"
#include "DataLogger.h"

unsigned long lastTime;
unsigned long currentTime;

KalmanFilter kalmanX, kalmanY, kalmanZ;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  initializeSensors();
  initializeSDCard();
  createLogFile();

  lastTime = millis();
}

void loop() {
  currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  SensorData sensorData = readSensors(deltaTime, kalmanX, kalmanY, kalmanZ);

  logData(sensorData);

  delay(100);
}