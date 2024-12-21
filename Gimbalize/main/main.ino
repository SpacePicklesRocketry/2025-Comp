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
// Serial.print("Accel (m/s^2): ");
// Serial.print("X: "); Serial.print(currentData.accelX);
// Serial.print(", Y: "); Serial.print(currentData.accelY);
// Serial.print(", Z: "); Serial.println(currentData.accelZ);

Serial.print("Raw Accel (m/s^2): ");
// Serial.print("X: "); Serial.print(accelX);
// Serial.print(", Y: "); Serial.print(accelY);
Serial.print(", ZRAW: "); Serial.println(accelZ);

// Serial.print("Gyro (deg/s): ");
// Serial.print("X: "); Serial.print(currentData.gyroX);
// Serial.print(", Y: "); Serial.print(currentData.gyroY);
// Serial.print(", Z: "); Serial.println(currentData.gyroZ);

    previousData = currentData;
    delay(100);
}