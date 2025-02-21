#include "Parachute.h"
#include "Sensors.h"
#include <Servo.h>

#define PARACHUTE_SERVO_PIN 10  // Adjust as needed
#define PARACHUTE_CLOSED_ANGLE 0  // Closed position
#define PARACHUTE_OPEN_ANGLE 90  // Fully open position

static unsigned long deploymentOffset = 0;
ParachuteStatus parachuteStatus = { false };
Servo parachuteServo;

void initializeParachute() {
    parachuteServo.attach(PARACHUTE_SERVO_PIN);
    parachuteServo.write(PARACHUTE_CLOSED_ANGLE);
    parachuteStatus.parachuteDeployed = false;
}

void updateParachute(const SensorData& sensorData) {
    if (!parachuteStatus.parachuteDeployed) {
        if (sensorData.apogeeDetected) {  // Ensure apogee has been detected
            unsigned long deploymentTime = sensorData.apogeeTime + deploymentOffset;

            if (deploymentOffset > 0 && millis() >= deploymentTime) {
                deployParachute();
            }
        }
    }
}

void setDeploymentDelayParachute(float delaySeconds) {
    deploymentOffset = static_cast<unsigned long>(delaySeconds * 1000);
}

void deployParachute() {
    parachuteStatus.parachuteDeployed = true;
    parachuteServo.write(PARACHUTE_OPEN_ANGLE);
}