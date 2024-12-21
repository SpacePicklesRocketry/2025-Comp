#include "Sensors.h"
#include <MKRIMU.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <SimpleKalmanFilter.h>

// Constants for BMP sensors
#define BMP1_ADDR 0x76 // Address of the first BMP390
#define BMP2_ADDR 0x77 // Address of the second BMP390
#define SEALEVELPRESSURE_HPA 1013.25 // Standard sea-level pressure in hPa
#define APOGEE_THRESHOLD -0.5 // Apogee detection threshold: Negative rate of climb

// Instances for BMP sensors
Adafruit_BMP3XX bmp1;
Adafruit_BMP3XX bmp2;

// Define Kalman filter for altitude
SimpleKalmanFilter altitudeFilter(2.0, 2.0, 0.5); // Measurement error, Estimate error, Process noise

// Variables for altitude and rate of climb
float previousAltitude = 0;
float currentAltitude = 0;

void initializeSensors() {
    if (!IMU.begin()) {
        Serial.println("Failed to initialize MKRIMU");
        while (1) delay(10); // Halt on failure
    }
    Serial.println("MKRIMU Initialized");

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

    if (!bmp1.begin_I2C(BMP1_ADDR) && !bmp2.begin_I2C(BMP2_ADDR)) {
        Serial.println("Both BMP sensors failed to initialize. Check wiring.");
        while (1);
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

    Serial.println("Initialization complete.");
}

SensorData readSensors(float deltaTime, SensorData& previousData) {
    SensorData data;

    float accelX_raw = 0, accelY_raw = 0, accelZ_raw = 0;
    float gyroX_raw = 0, gyroY_raw = 0, gyroZ_raw = 0;

    // Read gyroscope data
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX_raw, gyroY_raw, gyroZ_raw);
        

        // Convert gyroscope readings to degrees per second
        data.gyroX = gyroX_raw * 180.0 / M_PI;
        data.gyroY = gyroY_raw * 180.0 / M_PI;
        data.gyroZ = gyroZ_raw * 180.0 / M_PI;

        // Update roll, pitch, yaw using gyroscope data
        data.angleX = previousData.angleX + data.gyroX * deltaTime;
        data.angleY = previousData.angleY + data.gyroY * deltaTime;
        data.angleZ = previousData.angleZ + data.gyroZ * deltaTime;
    }

    // Read accelerometer data
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX_raw, accelY_raw, accelZ_raw);

        // Adjust raw accelerometer readings
        accelX_raw *= GRAVITY;
        accelY_raw *= GRAVITY;
        accelZ_raw *= GRAVITY;

        // Direct assignment to data fields
        data.accelX = accelX_raw;
        data.accelY = accelY_raw;
        data.accelZ = accelZ_raw;

        // Integrate acceleration to calculate velocity
        data.velocityX = previousData.velocityX + data.accelX * deltaTime;
        data.velocityY = previousData.velocityY + data.accelY * deltaTime;
        data.velocityZ = previousData.velocityZ + data.accelZ * deltaTime;

        // Integrate velocity to calculate position
        data.positionX = previousData.positionX + data.velocityX * deltaTime;
        data.positionY = previousData.positionY + data.velocityY * deltaTime;
        data.positionZ = previousData.positionZ + data.velocityZ * deltaTime;
    }

    // Read altitude from BMP sensor
    float rawAltitude = readAltitudeFromBMP();
    data.altitude = altitudeFilter.updateEstimate(rawAltitude);
    data.rateOfChange = data.altitude - previousAltitude;

    // Apogee detection
    if (data.rateOfChange < APOGEE_THRESHOLD) {
        Serial.println("Apogee detected!");
    }

    // Update previous altitude for the next iteration
    previousAltitude = data.altitude;

    // Debugging output
    // Serial.print("Altitude: ");
    // Serial.print(data.altitude);
    // Serial.print(" m, Rate of Climb: ");
    // Serial.println(data.rateOfChange);

    data.timestamp = millis();
    return data;
}


float readAltitudeFromBMP() {
    // Check both sensors and use the one that is available
    if (bmp1.performReading()) {
        return bmp1.readAltitude(SEALEVELPRESSURE_HPA);
    } else if (bmp2.performReading()) {
        return bmp2.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
        Serial.println("Failed to read altitude from BMP sensors.");
        return 0; // Default value in case of failure
    }
}