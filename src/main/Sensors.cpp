#include "Sensors.h"
#include <MKRIMU.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

#define BMP_ADDR 0x77
#define SEALEVELPRESSURE_HPA 1022   // TODO ADJUST
#define APOGEE_THRESHOLD -0.5

Adafruit_BMP3XX bmp;
SimpleKalmanFilter altitudeFilter(2.0, 2.0, 0.5);
SimpleKalmanFilter gyroFilter(2.0, 0.1, 0.01);

float previousAltitude = 0;
bool bmpWorking = true;

void configureBMP();
float readAltitude();

void initializeSensors() {
    Wire.begin();

    if (!IMU.begin()) {
        Serial.println("Failed to initialize MKRIMU");
        while (1) delay(10);
    }
    Serial.println("MKRIMU Initialized");

    // Initialize BMP sensor
    Serial.println("Initializing BMP390 sensor...");
    bmpWorking = bmp.begin_I2C(BMP_ADDR);
    if (bmpWorking) {
        configureBMP();
        Serial.println("BMP390 sensor initialized successfully.");
    } else {
        Serial.println("ERROR: Failed to initialize BMP390 sensor at address 0x77");
    }

    Serial.println("Initialization complete.");
}

void configureBMP() {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

SensorData readSensors(float deltaTime, SensorData &previousData) {
    SensorData data;

    float yaw, roll, pitch;
    float accelX_raw, accelY_raw, accelZ_raw;
    float gyroX_raw, gyroY_raw, gyroZ_raw;

    // Read Euler angles
    if (IMU.eulerAnglesAvailable()) {
        IMU.readEulerAngles(yaw, roll, pitch);
        data.angleX = roll;    // Roll
        data.angleY = pitch;   // Pitch
        data.angleZ = yaw;     // Yaw
    } else {
        data.angleX = previousData.angleX;
        data.angleY = previousData.angleY;
        data.angleZ = previousData.angleZ;
    }

    // Read gyroscope data
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX_raw, gyroY_raw, gyroZ_raw);
        gyroX_raw -= 0;        // Offset for X-axis
        gyroY_raw -= -1;       // Offset for Y-axis
        gyroZ_raw -= -1;       // Offset for Z-axis
        data.gyroX = gyroFilter.updateEstimate(gyroX_raw * 180.0 / M_PI);
        data.gyroY = gyroFilter.updateEstimate(gyroY_raw * 180.0 / M_PI);
        data.gyroZ = gyroFilter.updateEstimate(gyroZ_raw * 180.0 / M_PI);
    } else {
        data.gyroX = previousData.gyroX;
        data.gyroY = previousData.gyroY;
        data.gyroZ = previousData.gyroZ;
    }

    // Read accelerometer data
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX_raw, accelY_raw, accelZ_raw);
        accelX_raw -= 0.006;   // Offset for X-axis
        accelY_raw -= 0.050;   // Offset for Y-axis
        accelZ_raw += 0.033;   // Offset for Z-axis
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

    // Calculate velocity and position
    data.velocityX = previousData.velocityX + data.accelX * deltaTime;
    data.velocityY = previousData.velocityY + data.accelY * deltaTime;
    data.velocityZ = previousData.velocityZ + data.accelZ * deltaTime;
    data.positionX = previousData.positionX + data.velocityX * deltaTime;
    data.positionY = previousData.positionY + data.velocityY * deltaTime;
    data.positionZ = previousData.positionZ + data.velocityZ * deltaTime;

    // Read altitude
    data.altitude = readAltitude();
    data.rateOfChange = data.altitude - previousAltitude;

    Serial.print("Altitude: ");
    Serial.println(data.altitude);

    previousAltitude = data.altitude;
    data.timestamp = millis();
    return data;
}

float readAltitude() {
    if (!bmpWorking) {
        Serial.println("BMP390 sensor not functional. Attempting reinitialization...");
        bmpWorking = bmp.begin_I2C(BMP_ADDR);
        if (bmpWorking) {
            configureBMP();
            Serial.println("BMP390 reinitialized successfully.");
        } else {
            Serial.println("ERROR: Reinitialization failed. Sensor may be disconnected.");
            return -1;
        }
    }

    if (!bmp.performReading()) {
        Serial.println("ERROR: Failed to read from BMP390 sensor.");
        bmpWorking = false;
        return -1;
    }

    return bmp.readAltitude(SEALEVELPRESSURE_HPA);
}