#include <airbrake.h>
#include <Sensors.cpp>
#include <Servo.h>

Servo servo1;
Servo servo2;

airbrake1_pin = 8; //CHANGE TO CORRECT PIN!!!
airbrake2_pin = 9; //CHANGE TO CORRECT PIN!!!
alt_threshold = 182.88; //Meters - CHANGE TO CORRECT THRESHOLD!!! 182.88 meters = 600 feet
airbrake_deployed = false;
airbrake_open = 150; //CHANGE TO CORRECT ANGLE!!!
airbrake_close = -3; //CHANGE TO CORRECT ANGLE!!!

void setup(){
    servo1.attach(airbrake1_pin);
    servo2.attach(airbrake2_pin);
    initializeSensors();
    servo1.write(airbrake_close);
    servo2.write(airbrake_close);
}

void openAirbrake(bool &airbrake_deployed){
    if (readAltitudeFromBMP() >= alt_threshold && airbrake_deployed == false){
        servo1.write(airbrake_open);
        servo2.write(airbrake_open);
        airbrake_deployed = true;
    }
}

void closeAirbrake(bool &airbrake_deployed){
    if (readAltitudeFromBMP() < alt_threshold && airbrake_deployed == true){
        servo1.write(airbrake_close);
        servo2.write(airbrake_close);
        airbrake_deployed = false;
    }
}

