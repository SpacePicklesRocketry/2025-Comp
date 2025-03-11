#include "Airbrake.h"
#include "Sensors.h"
#include <Servo.h>

#define AIRBRAKE_SERVO_PIN 5 //TODO
#define AIRBRAKE_FLUSH_ANGLE 122  // flush
#define AIRBRAKE_DEPLOY_ANGLE 55  // deploy

static unsigned long deploymentOffset = 0; 
AirbrakeStatus airbrakeStatus = { false };
Servo airbrakeServo;

void initializeAirbrake() {
    airbrakeServo.attach(AIRBRAKE_SERVO_PIN);
    airbrakeServo.write(AIRBRAKE_FLUSH_ANGLE);
    airbrakeStatus.airbrakesDeployed = false;
}

void updateAirbrake(const SensorData& sensorData) {
    if (!airbrakeStatus.airbrakesDeployed) {
        if (sensorData.liftoffDetected) {
            unsigned long deploymentTime = sensorData.liftoffTime + deploymentOffset;
            
            if (deploymentOffset > 0 && millis() >= deploymentTime) {
                deployAirbrakes();
            }
        }
    }
}

void setDeploymentDelayAirbrake(float delaySeconds) {
    deploymentOffset = static_cast<unsigned long>(delaySeconds * 1000);
}

void deployAirbrakes() {
    airbrakeStatus.airbrakesDeployed = true;
    airbrakeServo.write(AIRBRAKE_DEPLOY_ANGLE);
}