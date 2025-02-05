#include <airbrake.h>
#include <Sensors.cpp>
#include <Servo.h>


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

