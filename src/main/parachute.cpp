#include <Servo.h>
#include<parachute.h>
#include <Sensors.cpp>

Servo door_servo;
Servo parachute_servo;

door_servo_pin = 10; //CHANGE TO CORRECT PIN!!!
parachute_servo_pin = 11; //CHANGE TO CORRECT PIN!!!
alt_threshold = 182.88; //Meters - CHANGE TO CORRECT THRESHOLD!!! 182.88 meters = 600 feet

altitude_passed = false;
parachute_deployed = false;

door_open = 150; //CHANGE TO CORRECT ANGLE!!!
parachute_open = 130; //CHANGE TO CORRECT ANGLE!!!
close_servos = 0;

void setup(){
    door_servo.attach(door_servo_pin);
    parachute_servo.attach(parachute_servo_pin);
    initializeSensors();
    door_servo.write(close_servos);
    parachute_servo.write(close_servos);
}

void deploy_parachute(bool &altitude_passed, bool &parachute_deployed){
    if(altitude_passed == false && readAltitudeFromBMP() >= alt_threshold){
        altitude_passed = true;
        if (altitude_passed == true && readAltitudeFromBMP() <= alt_threshold && door_opened == false){
            door_servo.write(door_open);
            delay(2000);
            parachute_servo.write(parachute_open);
            delay(5000)
            parachute_deployed = true;
        } else{
            parachute_deployed = false;
        }
    } else {
        altitude_passed = false;
    }
}

void close_parachute_servos(bool &altitude_passed, bool &parachute_deployed){
    if(parachute_deployed == true && readAltitudeFromBMP() <= alt_threshold){
        door_servo.write(close_servos);
        parachute_servo.write(close_servos);
        parachute_deployed = false;
        altitude_passed = false;
    } else {
        parachute_deployed = true;
        altitude_passed = true;
    }
}