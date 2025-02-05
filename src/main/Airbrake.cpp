#include <Airbrake.h>
#include <Sensors.cpp>
#include <Servo.h>

Servo airbrakeServo;
int airbrakePin = 9; // CHANGE TO CORRECT PIN!!!
int airbrakeOpen = 180; // CHANGE TO CORRECT ANGLE!!!
int airbrakeClose = -2; // CHANGE TO CORRECT ANGLE!!!
int altitudeThreshold = 182.88; //Meters - CHANGE TO CORRECT THRESHOLD!!! 182.88 meters = 600 feet

bool airbrakeDeployed = false;

float currentAltitude = readAltitudeFromBMP();



void initializeAirbrake(){
    airbrakeServo.attach(airbrakePin);
    airbrakeServo.write(airbrakeClose);
}

void openAirbrake(){
    if (currentAltitude >= altitudeThreshold && airbrakeDeployed == false){
        airbrakeServo.write(airbrakeOpen);
        airbrakeDeployed = true;
    } else {
        airbrakeDeployed = false;
    }
}

void closeAirbrake(){
    if (currentAltitude <= altitudeThreshold && airbrakeDeployed == true){
        airbrakeServo.write(airbrakeClose);
        airbrakeDeployed = false;
    } else {
        airbrakeDeployed = true;
    }
}

