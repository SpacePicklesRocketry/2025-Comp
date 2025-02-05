#include <airbrake.h>
#include <Sensors.cpp>
#include <Servo.h>

Servo airbrakeServo;

const int airbrakePin = 8; //TODO
altitudeThreshold = 182.88; //Meters - CHANGE TO CORRECT THRESHOLD!!! 182.88 meters = 600 feet

airbrakeDeployed = false;

const int airbrakeOpen = 150; //TODO
const int airbrakeClose = -3; //TODO

void initializeAirbrake(){
    airbrakeServo.attach(airbrakePin);
    airbrakeServo.write(airbrake_close);
}

void openAirbrake(bool &airbrake_deployed){
    if (readAltitudeFromBMP() >= altitudeThreshold && airbrake_deployed == false){
        airbrakeServo.write(airbrakeOpen);
        airbrakeDeployed = true;
    } else {
        airbrakeDeployed = false;
    }
}

void closeAirbrake(bool &airbrake_deployed){
    if (readAltitudeFromBMP() < altitudeThreshold && airbrakeDeployed == true){
        airbrakeServo.write(airbrakeClose);
        airbrakeDeployed = false;
    } else {
        airbrakeDeployed = true;
    }
}

