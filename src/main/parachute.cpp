#include <Servo.h>
#include<parachute.h>
#include <Sensors.cpp>

Servo doorServo;
Servo parachuteServo;

int doorServoPin = 10; //TODO: CHANGE TO CORRECT PIN!!!
int parachuteServoPin = 11; //CHANGE TO CORRECT PIN!!!
float altThreshold = 182.88; //Meters - CHANGE TO CORRECT THRESHOLD!!! 182.88 meters = 600 feet

float altitude = readAltitudeFromBMP();
bool altitudePassed = false;
bool parachuteDeployed = false;
bool doorOpened = false;

int doorOpen = 150; //CHANGE TO CORRECT ANGLE!!!
int closeServos = -2;

void initializeParachute(){
    doorServo.attach(doorServoPin);
    initializeSensors();
    doorServo.write(closeServos);
}

void deploy_parachute(bool altitudePassed, bool parachuteDeployed){
    if(altitudePassed == false && altitude >= altThreshold){
        altitudePassed = true;
        if (altitudePassed == true && altitude <= altThreshold && doorOpened == false){
            doorServo.write(door_open);
            parachuteDeployed = true;
            delay(3000);
        } else{
            parachuteDeployed = false;
        }
    } else {
        altitudePassed = false;
    }
}

void close_parachute_servo(bool altitudePassed, bool parachuteDeployed){
    if(parachuteDeployed == true && altitude <= altThreshold){
        door_servo.write(closeServos);;
        parachuteDeployed = false;
        altitudePassed = false;
    } else {
        parachuteDeployed = true;
        altitudePassed = true;
    }
}