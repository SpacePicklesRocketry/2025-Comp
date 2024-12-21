#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP180.h>
#include <stage_detection.h>
#include <math.h>

// Set Starting Phase
current_stage = FlightPhase.GROUND;


// Initialize sensors
Adafruit_MPU6050 mpu;
Adafruit_BMP180 bmp180;

void initializeSensors() {
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU chip!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  if (!bmp.begin()) {
    Serial.println("Failed to find bmp180 sensor!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BMP180 Found!");
}

// Main loop
void detectFlightPhase_bmp180(float accel_x, float accel_y, float accel_z, float altitude, float current_pressure, float last_pressure){

    //Create sensor event and get raw data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Read acceleration data from MPU6050
    mpu.getAcceleration(accel_x, accel_y, accel_z);

    // Read altitude data from BMP180
    altitude = bmp180.readAltitude();

    
    // Pressure at different times to check difference in height.
    current_pressure = 1013;
    last_pressure = 1013; 
    
    // Detect flight phases based on sensor data and thresholds
    if(current_stage == FlightPhase.GROUND && accel_z >= launch_accel_threshold){
            Serial.println("Launch detected!");
            current_stage = FlightPhase.POWERED_ASCENT;
    } else if(current_stage == FlightPhase.POWERED_ASCENT && accel_z > coast_accel_threshold && current_pressure-last_pressure < -1 ){
            Serial.println("Coasting Phase detected!");
            current_stage = FlightPhase.COASTING;
    } else if(current_stage == FlightPhase.COASTING && abs(altitude - apogee_altitude_threshold)<10 && accel_z < 5){
            Serial.println("Apogee detected!");
            current_stage = FlightPhase.APOGEE;
    } else if(current_stage == FlightPhase.APOGEE && 0 > accel_z && altitude < apogee_altitude_threshold ){
            Serial.println("Descent detected!");
            current_stage = FlightPhase.DESCENT;
    } else if(current_stage == FlightPhase.DESCENT && abs(0 - accel_z) < 2){ // <2 is accounting for hallucination data
            Serial.println("Landing detected!");
            current_stage = FlightPhase.LANDING;
    } else {
            Serial.println("PROBLEM DETECTED!");
      }
        
    

    lastPressure = currentPressure;
    currentPressure = bmp180.readPressure()/100 //hectopascals

    // Delay for sensor readings
    delay(100); 
}
