#include "Sensors.h"
#include "DataLogger.h"

// Global variables
SensorData previousData = {};

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial);

    // Initialize sensors and SD card
    initializeSensors();
    initializeSDCard();
    createLogFile();

    Serial.println("System Initialized");
}

void loop() {
    // Delta time calculation
    static unsigned long lastTime = 0;
    if (lastTime == 0) lastTime = millis(); // Initialize on first iteration

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) * 0.001; // Convert to seconds
    lastTime = currentTime;

    // Read sensors and log data
    SensorData currentData = readSensors(deltaTime, previousData);
    logData(currentData);

    // Debugging output

    Serial.print("Accel (m/s^2): ");
    Serial.print("X: "); Serial.print(currentData.accelX);
    Serial.print(", Y: "); Serial.print(currentData.accelY);
    Serial.print(", Z: "); Serial.println(currentData.accelZ);

    // Serial.print("Gyro (deg/s): ");
    // Serial.print("X: "); Serial.print(currentData.gyroX);
    // Serial.print(", Y: "); Serial.print(currentData.gyroY);
    // Serial.print(", Z: "); Serial.println(currentData.gyroZ);

    // Serial.print("Angle (deg): ");
    // Serial.print("X: "); Serial.print(currentData.angleX);
    // Serial.print(", Y: "); Serial.print(currentData.angleY);
    // Serial.print(", Z: "); Serial.println(currentData.angleZ);

    // Serial.print("Position (m): ");
    // Serial.print("X: "); Serial.print(currentData.positionX);
    // Serial.print(", Y: "); Serial.print(currentData.positionY);
    // Serial.print(", Z: "); Serial.println(currentData.positionZ);

    // Update previous data for the next loop

    delay(100); // Small delay for stability
}