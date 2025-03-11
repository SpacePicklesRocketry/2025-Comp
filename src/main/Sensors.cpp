#include "Sensors.h"
#include <MKRIMU.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>

// #define BMP1_ADDR 0x76                // Address of the first BMP390
#define BMP2_ADDR 0x77                // Address of the second BMP390
#define SEALEVELPRESSURE_HPA 1019     // Standard sea-level pressure in hPa
#define APOGEE_DETECTION_THRESHOLD -0.5  // m/s, threshold for apogee detection
#define MOVING_AVG_WINDOW 10          // Number of samples for moving average

// Adafruit_BMP3XX bmp1;
Adafruit_BMP3XX bmp2;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Globals
float previousAltitude = 0;
float baseAltitude = 0;  
bool altitudeZeroed = false;  
float initialRoll = 0, initialPitch = 0, initialYaw = 0;
unsigned long initialTime;

// Moving average buffer for altitude
float altitudeHistory[MOVING_AVG_WINDOW] = {0};
int altitudeIndex = 0;
bool bufferFilled = false;

// **Function Prototype**
float computeMovingAverage(float newAltitude);

void initializeSensors() {
    // Initialize IMU
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055");
        while (1) delay(10);
    }
    Serial.println("BNO055 Initialized");

    // Initialize BMP sensors
    // if (!bmp1.begin_I2C(BMP1_ADDR)) {
    //     Serial.println("Sensor 1 not found at address 0x76");
    // } else {
    //     Serial.println("Sensor 1 initialized at address 0x76");
    // }

    if (!bmp2.begin_I2C(BMP2_ADDR)) {
        Serial.println("Sensor 2 not found at address 0x77");
    } else {
        Serial.println("Sensor 2 initialized at address 0x77");
    }

    // // Configure BMP sensors
    // if (bmp1.begin_I2C(BMP1_ADDR)) {
    //     bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    //     bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    //     bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    //     bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
    // }

    if (bmp2.begin_I2C(BMP2_ADDR)) {
        bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
    }

    delay(2000); // Allow sensors to stabilize

    // **Discard the first few readings**
    float stableAltitude = 0;
    for (int i = 0; i < 5; i++) {
        stableAltitude = readAltitudeFromBMP();
        Serial.print("Discarding altitude sample: ");
        Serial.println(stableAltitude, 2);
        delay(200);
    }

    // Set baseline altitude
    baseAltitude = readAltitudeFromBMP();
    Serial.print("Base altitude set to: ");
    Serial.println(baseAltitude, 2);

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
    data.gyroX = gyro.x();
    data.gyroY = gyro.y();
    data.gyroZ = gyro.z();

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

    // Read altitude, apply moving average filter
    float rawAltitude = readAltitudeFromBMP();
    data.altitude = (rawAltitude - baseAltitude);
    //data.altitude = computeMovingAverage(rawAltitude - baseAltitude);
    data.rateOfChange = (data.altitude - previousAltitude) / deltaTime;

    // Liftoff detection
    if (!previousData.liftoffDetected && (data.accelZ > LIFTOFF_ACCELERATION_THRESHOLD)) {
        data.liftoffDetected = true;
        data.liftoffTime = millis();
        Serial.print("Liftoff detected! LiftTime: ");
        Serial.println(data.liftoffTime);
    } else {
        data.liftoffDetected = previousData.liftoffDetected;
        data.liftoffTime = previousData.liftoffTime;
    }

    // Apogee detection (only after liftoff)
    if (data.liftoffDetected && !previousData.apogeeDetected && (data.rateOfChange < APOGEE_DETECTION_THRESHOLD)) {
        data.apogeeDetected = true;
        data.apogeeTime = millis();
        Serial.print("Apogee detected! Apogee Time: ");
        Serial.println(data.apogeeTime);
    } else {
        data.apogeeDetected = previousData.apogeeDetected;
        data.apogeeTime = previousData.apogeeTime;
    }

    previousAltitude = data.altitude;
    data.timestamp = millis() - initialTime;
    return data;
}

float readAltitudeFromBMP() {
    // if (bmp1.performReading()) {
    //     return bmp1.readAltitude(SEALEVELPRESSURE_HPA);
    // } else if (bmp2.performReading()) {
    //     return bmp2.readAltitude(SEALEVELPRESSURE_HPA);
    // } else {
    //     return 0;  // Fallback if both sensors fail
    // }

    if(bmp2.performReading()){
      return bmp2.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      return 0;
    }
}

// **Moving Average Filter for Altitude**
float computeMovingAverage(float newAltitude) {
    altitudeHistory[altitudeIndex] = newAltitude;
    altitudeIndex = (altitudeIndex + 1) % MOVING_AVG_WINDOW;

    if (!bufferFilled && altitudeIndex == 0) {
        bufferFilled = true;  // Flag when buffer is full
    }

    float sum = 0;
    int count = bufferFilled ? MOVING_AVG_WINDOW : altitudeIndex;
    for (int i = 0; i < count; i++) {
        sum += altitudeHistory[i];
    }
    return sum / count;
}