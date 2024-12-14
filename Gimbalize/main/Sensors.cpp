#include "Sensors.h"
#include <MKRIMU.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>

Adafruit_BMP3XX bmp;

void initializeSensors() {
  Wire.begin();

    
  if (!IMU.begin()) {
    Serial.println("Failed to find MKRIMU chip");
    while (1) {
      delay(10);
    }

  }
  Serial.println("MKRIMU Found!");

  // if (!bmp.begin_I2C()) {
  //   Serial.println("Failed to find BMP390 sensor");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // Serial.println("BMP390 Found!");

}

SensorData readSensors(float deltaTime, KalmanFilter& kalmanX, KalmanFilter& kalmanY, KalmanFilter& kalmanZ) {
  SensorData data;

  if (IMU.gyroscopeAvailable()) {
    float gyroX, gyroY, gyroZ;
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Calculate raw angles from gyroscope
    float rawAngleX = gyroX * deltaTime;
    float rawAngleY = gyroY * deltaTime;
    float rawAngleZ = gyroZ * deltaTime;

    // Use Kalman filter to compute filtered angles
    data.angleX = kalmanX.update(rawAngleX, gyroX, deltaTime);
    data.angleY = kalmanY.update(rawAngleY, gyroY, deltaTime);
    data.angleZ = kalmanZ.update(rawAngleZ, gyroZ, deltaTime);
  }

  if (IMU.accelerationAvailable()) {
    float accelX, accelY, accelZ;
    IMU.readAcceleration(accelX, accelY, accelZ);

    // accelerometer-based calculations TODO
  }
  // data.pressure = bmp.readPressure();
  // data.altitude = bmp.readAltitude(1013.25);

  // Add a timestamp for the current reading
  data.timestamp = millis();

  return data;
}