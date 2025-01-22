#include "Sensors.h"
#include <MKRIMU.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <SimpleKalmanFilter.h>

#define BMP1_ADDR 0x76 // Address of the first BMP390
#define BMP2_ADDR 0x77 // Address of the second BMP390
#define SEALEVELPRESSURE_HPA 1013.25 // Standard sea-level pressure in hPa
#define APOGEE_THRESHOLD -0.5 // Apogee detection threshold: Negative rate of climb


Adafruit_BMP3XX bmp1;
Adafruit_BMP3XX bmp2;

SimpleKalmanFilter altitudeFilter(2.0, 2.0, 0.5);
SimpleKalmanFilter gyroFilter(2.0, 0.1, 0.01);

float previousAltitude = 0;

void initializeSensors() {
    if (!IMU.begin()) {
        Serial.println("Failed to initialize MKRIMU");
        while (1) delay(10);
    }
    Serial.println("MKRIMU Initialized");

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

    if (!bmp1.begin_I2C(BMP1_ADDR) && !bmp2.begin_I2C(BMP2_ADDR)) {
        Serial.println("Both BMP sensors failed to initialize. Check wiring.");
        while (1);
    }

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

    Serial.println("Initialization complete.");
}

SensorData readSensors(float deltaTime, SensorData& previousData) {
    SensorData data;

    float yaw, roll, pitch;
    float accelX_raw, accelY_raw, accelZ_raw;
    float gyroX_raw, gyroY_raw, gyroZ_raw;

    if (IMU.eulerAnglesAvailable()) {
        IMU.readEulerAngles(yaw, roll, pitch);

        data.angleX = roll;    // Roll
        data.angleY = pitch;   // Pitch
        data.angleZ = yaw; // Yaw

    } else {
        data.angleX = previousData.angleX;
        data.angleY = previousData.angleY;
        data.angleZ = previousData.angleZ;
    }

    if (IMU.gyroscopeAvailable()){
      IMU.readGyroscope(gyroX_raw, gyroY_raw, gyroZ_raw);

      gyroX_raw -= 0;       // Offset for X-axis
      gyroY_raw -= -1;      // Offset for Y-axis
      gyroZ_raw -= -1;      // Offset for Z-axis
      
      data.gyroX = gyroFilter.updateEstimate(gyroX_raw * 180.0 / M_PI);
      data.gyroY = gyroFilter.updateEstimate(gyroY_raw * 180.0 / M_PI);
      data.gyroZ = gyroFilter.updateEstimate(gyroZ_raw * 180.0 / M_PI);
    } else {
      data.gyroX = previousData.gyroX;
      data.gyroY = previousData.gyroY;
      data.gyroZ = previousData.gyroZ;
    }

    if (IMU.accelerationAvailable()){
      IMU.readAcceleration(accelX_raw, accelY_raw, accelZ_raw);

      accelX_raw -= 0.006;       // Offset for X-axis
      accelY_raw -= 0.050;      // Offset for Y-axis
      accelZ_raw += 0.033;      // Offset for Z-axis

      data.accelX = (accelX_raw * GRAVITY);
      data.accelY = (accelY_raw * GRAVITY);
      data.accelZ = (accelZ_raw * GRAVITY);

      data.accelX -= GRAVITY * sin(data.angleY * M_PI / 180.0);
      data.accelY += GRAVITY * sin(data.angleX * M_PI / 180.0);
      data.accelZ -= GRAVITY * cos(data.angleY * M_PI / 180.0) * cos(data.angleX * M_PI / 180.0);

    } else {
      data.accelX = previousData.accelX;
      data.accelY = previousData.accelY;
      data.accelZ = previousData.accelZ;
    }

    data.velocityX = previousData.velocityX + data.accelX * deltaTime;
    data.velocityY = previousData.velocityY + data.accelY * deltaTime;
    data.velocityZ = previousData.velocityZ + data.accelZ * deltaTime;

    data.positionX = previousData.positionX + data.velocityX * deltaTime; // + (0.5)*(data.accelX)*(deltaTime)*(deltaTime);
    data.positionY = previousData.positionY + data.velocityY * deltaTime; // + (0.5)*(data.accelY)*(deltaTime)*(deltaTime);
    data.positionZ = previousData.positionZ + data.velocityZ * deltaTime; // + (0.5)*(data.accelZ)*(deltaTime)*(deltaTime);
    

    float rawAltitude = readAltitudeFromBMP();
    data.altitude = altitudeFilter.updateEstimate(rawAltitude);
    data.rateOfChange = data.altitude - previousAltitude;

    if (data.rateOfChange < APOGEE_THRESHOLD) {
        Serial.println("Apogee detected!");
    }

    previousAltitude = data.altitude;

    data.timestamp = millis();
    return data;
}

float readAltitudeFromBMP() {
    if (bmp1.performReading()) {
        return bmp1.readAltitude(SEALEVELPRESSURE_HPA);
    } else if (bmp2.performReading()) {
        return bmp2.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
        Serial.println("Failed to read altitude from BMP sensors.");
        return 0;
    }
}

bool checkStageSepetation() {
    if (analogRead(21)>500){
        return false;
    }
    else if (analogRead(21)<200){
        return true;
    }
    else{
        Serial.println("Flaky connection check stage sensor");
        return false;
    }
}