#include "Sensors.h"
#include "DataLogger.h"
// #include "Airbrake.h"
// #include "Parachute.h"

SensorData previousData = {};

void setup() {
    Serial.begin(115200);
    while (!Serial);

    unsigned long startTime = millis();
    // Serial.print("Program Start Time: ");
    // Serial.println(startTime);

    initializeSensors();
    calibrateSensors();
    // initializeAirbrake();
    // setDeploymentDelayAirbrake(5);
//     initializeParachute();
//     setDeploymentDelayParachute(0);
    initializeSDCard();
    createLogFile();

    // Serial.println("System Initialized");
}

void loop() {
    digitalWrite(LED_BUILTIN, 1);
    // Delta time calculation
    static unsigned long lastTime = 0;
    if (lastTime == 0) lastTime = millis();

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) * .001;
    lastTime = currentTime;

    SensorData currentData = readSensors(deltaTime, previousData);
    
    // updateAirbrake(currentData);
//     updateParachute(currentData);
    logData(currentData);

    previousData = currentData;

    Serial.println(currentData.altitude);
    Serial.print(currentData.apogeeDetected);
    if (currentData.apogeeDetected){
      Serial.println(currentData.apogeeTime);
    }

    if (currentData.liftoffDetected) { 
        Serial.print("Liftoff detected! LiftTime: ");
        Serial.println(currentData.liftoffTime);
    }

    Serial.print("Accel (m/s^2): ");
    Serial.print("X: "); Serial.print(currentData.accelX);
    Serial.print(", Y: "); Serial.print(currentData.accelY);
    Serial.print(", Z: "); Serial.println(currentData.accelZ);

    Serial.print("Gyro (deg/s): ");
    Serial.print("X: "); Serial.print(currentData.gyroX);
    Serial.print(", Y: "); Serial.print(currentData.gyroY);
    Serial.print(", Z: "); Serial.println(currentData.gyroZ);

    Serial.print("Angle (deg): ");
    Serial.print("Roll: "); Serial.print(currentData.roll);
    Serial.print(", Pitch: "); Serial.print(currentData.pitch);
    Serial.print(", Yaw: "); Serial.println(currentData.yaw);

    Serial.print("Speed (m/s: ");
    Serial.print("X: "); Serial.print(currentData.velocityX);
    Serial.print(", Y: "); Serial.print(currentData.velocityY);
    Serial.print(", Z: "); Serial.println(currentData.velocityZ);

    Serial.print("Position (m): ");
    Serial.print("X: "); Serial.print(currentData.positionX);
    Serial.print(", Y: "); Serial.print(currentData.positionY);
    Serial.print(", Z: "); Serial.println(currentData.positionZ);

    Serial.print("GPS Data: ");
    Serial.print("Latitude: "); Serial.print(currentData.latitude, 7);
    Serial.print(", Longitude: "); Serial.print(currentData.longitude, 7);
    Serial.print(", Satellites "); Serial.println(currentData.satellites, 7);

    Serial.println(" ");
    digitalWrite(LED_BUILTIN, 0);
}