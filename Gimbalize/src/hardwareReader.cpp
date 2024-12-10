#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

File dataFile;

float angleX = 0;
float angleY = 0;
float angleZ = 0;

unsigned long lastTime;
unsigned long currentTime;

#define SD_CS_PIN SDCARD_SS_PIN

// Kalman filter variables
float Q_angle = 0.001; // Process noise variance for the angle
float Q_gyro = 0.003;  // Process noise variance for the gyro bias
float R_angle = 0.03;  // Measurement noise variance
float biasX = 0, biasY = 0, biasZ = 0;
float P[3][2][2] = {0}; // Error covariance matrix for X, Y, Z
float angleKalmanX = 0, angleKalmanY = 0, angleKalmanZ = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Wire.begin();

  Serial.println("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Done.");

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

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("SD card initialized!");

  dataFile = SD.open("data_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time (ms), Angle X (deg), Angle Y (deg), Angle Z (deg), Temperature (C), Pressure (Pa), Altitude (m)");
    dataFile.close();
  } else {
    Serial.println("Error opening file!");
  }

  lastTime = millis();
}

float applyKalmanFilter(float newAngle, float newRate, float& angle, float& bias, float P[2][2], float deltaTime) {
  // Prediction step
  angle += deltaTime * (newRate - bias);
  P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= deltaTime * P[1][1];
  P[1][0] -= deltaTime * P[1][1];
  P[1][1] += Q_gyro * deltaTime;

  // Update step
  float S = P[0][0] + R_angle; // Estimate error
  float K[2]; // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle; // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;

  // Update error covariance
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

void loop() {
  currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rawAngleX = g.gyro.x * deltaTime * 180.0 / PI;
  float rawAngleY = g.gyro.y * deltaTime * 180.0 / PI;
  float rawAngleZ = g.gyro.z * deltaTime * 180.0 / PI;

  angleKalmanX = applyKalmanFilter(rawAngleX, g.gyro.x, angleKalmanX, biasX, P[0], deltaTime);
  angleKalmanY = applyKalmanFilter(rawAngleY, g.gyro.y, angleKalmanY, biasY, P[1], deltaTime);
  angleKalmanZ = applyKalmanFilter(rawAngleZ, g.gyro.z, angleKalmanZ, biasZ, P[2], deltaTime);

  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(1013.25);

  Serial.print("Kalman Angle X: ");
  Serial.print(angleKalmanX);
  Serial.print(", Kalman Angle Y: ");
  Serial.print(angleKalmanY);
  Serial.print(", Kalman Angle Z: ");
  Serial.print(angleKalmanZ);
  Serial.print(", Temperature: ");
  Serial.print(temperature);
  Serial.print(" C, Pressure: ");
  Serial.print(pressure);
  Serial.print(" Pa, Altitude: ");
  Serial.println(altitude);

  dataFile = SD.open("data_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(currentTime);
    dataFile.print(", ");
    dataFile.print(angleKalmanX);
    dataFile.print(", ");
    dataFile.print(angleKalmanY);
    dataFile.print(", ");
    dataFile.print(angleKalmanZ);
    dataFile.print(", ");
    dataFile.print(temperature);
    dataFile.print(", ");
    dataFile.print(pressure);
    dataFile.print(", ");
    dataFile.println(altitude);
    dataFile.close();
  } else {
    Serial.println("Error opening file for writing!");
  }

  delay(100);
}