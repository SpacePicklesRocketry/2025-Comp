#include <stage_detection_mkrimu.cpp>
#include <stage_detection_mpu6050_bmp180.cpp>
#include <stage_detection_mpu6050_bmp390.cpp>
#include <Wire.h>
#include <Adafruit_BMP180.h>
#include <Adafruit_BMP390.h>
#include <MKRIMU.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP180 bmp180;
Adafruit_BMP390 bmp390;

float bmp390 = false;
float bmp180 = false;
float mkrimu = false;

void sensor_online(){
    if(bmp390.begin()){
        bmp390 = true;
    } else if(bmp180.begin()){
        bmp180 = true;
    } else if(IMU.begin()){
        mkrimu = true;
    } else {
        Serial.println("No sensor data available!");
    }
}

void setup() {
    Serial.begin(9600);
    sensor_online();
}

void loop() {
    if(bmp390 == true){
        detectFlightPhase_bmp390(accel_x, accel_y, accel_z, altitude, current_pressure, last_pressure);
    } else if(bmp180 == true){
        detectFlightPhase_bmp180(accel_x, accel_y, accel_z, altitude, current_pressure, last_pressure);
    } else if(mkrimu == true){
         detectFlightPhase_mkrimu(accel_x, accel_y, accel_z, altitude, current_pressure, last_pressure);
    } else {
        Serial.println("No sensor data available!");
    }
}