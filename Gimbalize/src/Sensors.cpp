#include "Sensors.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

void initializeSensors() {
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  if (!bmp.begin()) {
    Serial.println("Failed to find BMP180 sensor");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BMP180 Found!");
}

SensorData readSensors(float deltaTime, KalmanFilter& kalmanX, KalmanFilter& kalmanY, KalmanFilter& kalmanZ) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rawAngleX = g.gyro.x * deltaTime * 180.0 / PI;
  float rawAngleY = g.gyro.y * deltaTime * 180.0 / PI;
  float rawAngleZ = g.gyro.z * deltaTime * 180.0 / PI;

  SensorData data;
  data.angleX = kalmanX.update(rawAngleX, g.gyro.x, deltaTime);
  data.angleY = kalmanY.update(rawAngleY, g.gyro.y, deltaTime);
  data.angleZ = kalmanZ.update(rawAngleZ, g.gyro.z, deltaTime);

  data.temperature = bmp.readTemperature();
  data.pressure = bmp.readPressure();
  data.altitude = bmp.readAltitude(1013.25);
  data.timestamp = millis();

  return data;
}