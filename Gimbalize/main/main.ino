#include "Sensors.h"
#include "DataLogger.h"

// Global variables
SensorData previousData = {};

void setup() {
    Serial.begin(115200);
    while (!Serial);

    initializeSensors();
    initializeSDCard();
    createLogFile();

    Serial.println("System Initialized");
}

void loop() {
    static unsigned long lastTime = millis();
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    SensorData currentData = readSensors(deltaTime, previousData);
    logData(currentData);

    float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

getRawData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

Serial.print("Accel (m/s^2): ");
Serial.print("X: "); Serial.print(accelX);
Serial.print(", Y: "); Serial.print(accelY);
Serial.print(", Z: "); Serial.println(accelZ);

Serial.print("Gyro (deg/s): ");
Serial.print("X: "); Serial.print(gyroX);
Serial.print(", Y: "); Serial.print(gyroY);
Serial.print(", Z: "); Serial.println(gyroZ);

    previousData = currentData;
    delay(100);
}