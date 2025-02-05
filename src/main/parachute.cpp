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

void setup(){
    doorServo.attach(doorServoPin);
    initializeSensors();
    doorServo.write(closeServos);
}

void deploy_parachute(bool altitude_passed, bool parachute_deployed){
    if(altitude_passed == false && altitude >= altThreshold){
        altitude_passed = true;
        if (altitude_passed == true && altitude <= altThreshold && doorOpened == false){
            door_servo.write(door_open);
            parachute_deployed = true;
            delay(3000);
        } else{
            parachute_deployed = false;
        }
    } else {
        altitude_passed = false;
    }
}

void close_parachute_servo(bool altitude_passed, bool parachute_deployed){
    if(parachute_deployed == true && altitude <= altThreshold){
        door_servo.write(closeServos);;
        parachute_deployed = false;
        altitude_passed = false;
    } else {
        parachute_deployed = true;
        altitude_passed = true;
    }
}