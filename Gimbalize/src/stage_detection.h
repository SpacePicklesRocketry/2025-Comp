#ifndef STAGE_DETECTION_H
#define STAGE_DETECTION_H

#include <Wire.h>
#include <MPU6050.h>
#include <BMP085.h>

// Define thresholds and other parameters
const float launch_accel_threshold = 75; // m/s^2
const float coast_accel_threshold = 40; // m/s^2
const float apogee_altitude_threshold = 240.792; // meters

//Different flight stages
enum FlightPhase {
    GROUND,
    POWERED_ASCENT,
    COASTING,
    APOGEE,
    DESCENT,
    LANDING
};

//Using variable instead
FlightPhase current_stage; 


// Functions
void initializeSensors();
void detectFlightPhase(float accel_x, float accel_y, float accel_z, float altitude, long current_pressure, long last_pressure);


#endif