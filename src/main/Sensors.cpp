#include "Sensors.h"
#include <MKRIMU.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <SimpleKalmanFilter.h>

#define BMP1_ADDR 0x76                // Address of the first BMP390
#define BMP2_ADDR 0x77                // Address of the second BMP390
#define SEALEVELPRESSURE_HPA 1013.25  // Standard sea-level pressure in hPa
#define APOGEE_THRESHOLD -0.5         // Apogee detection threshold: Negative rate of climb
#define GRAVITY 9.8                   // Acceleration due to gravity (m/s^2)

Adafruit_BMP3XX bmp1;
Adafruit_BMP3XX bmp2;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Filters
SimpleKalmanFilter altitudeFilter(2.0, 2.0, 0.5);
SimpleKalmanFilter gyroFilter(2.0, 0.1, 0.01);

// Globals
float previousAltitude = 0;
float initialRoll = 0, initialPitch = 0, initialYaw = 0;  // Store initial orientation for reference

unsigned long initialTime;

void initializeSensors() {
  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055");
    while (1) delay(10);
  }
  Serial.println("BNO055 Initialized");

  // Initialize BMP sensors
  if (!bmp1.begin_I2C(BMP1_ADDR)) {
    Serial.println("Sensor 1 not found at address 0x76");
  } else {
    Serial.println("Sensor 1 initialized at address 0x76");
  }

  if (!bmp2.begin_I2C(BMP2_ADDR)) {
    Serial.println("Sensor 2 not found at address 0x77");
  } else {
    Serial.println("Sensor 2 initialized at address 0x77");
  }

  // Configure BMP sensors
  if (bmp1.begin_I2C(BMP1_ADDR)) {
    bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  if (bmp2.begin_I2C(BMP2_ADDR)) {
    bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // Store initial IMU orientation
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  initialRoll = euler.x();
  initialPitch = euler.y();
  initialYaw = euler.z();

  initialTime = millis();

  Serial.println("Initialization complete.");
}

SensorData readSensors(float deltaTime, SensorData &previousData) {
  SensorData data;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  data.roll = euler.x() - initialRoll;
  data.pitch = euler.y() - initialPitch;
  data.yaw = euler.z() - initialYaw;

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data.gyroX = gyroFilter.updateEstimate(gyro.x());
  data.gyroY = gyroFilter.updateEstimate(gyro.y());
  data.gyroZ = gyroFilter.updateEstimate(gyro.z());

  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  data.accelX = linAccel.x();
  data.accelY = linAccel.y();
  data.accelZ = linAccel.z();

  data.velocityX = previousData.velocityX + data.accelX * deltaTime;
  data.velocityY = previousData.velocityY + data.accelY * deltaTime;
  data.velocityZ = previousData.velocityZ + data.accelZ * deltaTime;

  data.positionX = previousData.positionX + data.velocityX * deltaTime;
  data.positionY = previousData.positionY + data.velocityY * deltaTime;
  data.positionZ = previousData.positionZ + data.velocityZ * deltaTime;

  float rawAltitude = readAltitudeFromBMP();
  data.altitude = altitudeFilter.updateEstimate(rawAltitude);
  data.rateOfChange = (data.altitude - previousAltitude) / deltaTime;  // Proper rate of climb

  if (data.rateOfChange < APOGEE_THRESHOLD) {
    Serial.println("Apogee detected!");
  }

  previousAltitude = data.altitude;

  data.timestamp = millis() - initialTime;
  return data;
}

float readAltitudeFromBMP() {
  if (bmp1.performReading()) {
    return bmp1.readAltitude(SEALEVELPRESSURE_HPA);
  } else if (bmp2.performReading()) {
    return bmp2.readAltitude(SEALEVELPRESSURE_HPA);
  } else {
    return 0;  // Fallback if both sensors fail
  }
}
