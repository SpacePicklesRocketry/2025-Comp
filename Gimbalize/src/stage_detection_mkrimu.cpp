#include <Wire.h>
#include <stage_detection.h>
#include <math.h>
#include <MKRIMU.h>
#include <Adafruit_BMP3XX.h>
// Set Starting Phase
FlightPhase current_stage = FlightPhase.GROUND;

Adafruit_BMP3XX bmp390;

// Initialize sensor
void initializeSensors() {
  Wire.begin();

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("IMU initialized!");

  if (!bmp390.begin_I2C()) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1){
      delay(10);
    }
  }
  Serial.println("BMP390 initialized!");
}

// Main loop
void detectFlightPhase(float accel_x, float accel_y, float accel_z, float altitude, float current_pressure, float last_pressure){
  static unsigned long lastRun = 0;
  unsigned long currentRun = millis();
  // Delay for sensor readings
  if(currentRun - lastRun >= 100){ 
    lastRun = currentRun;

    // Read acceleration data from MPU6050
    IMU.readAcceleration(accel_x, accel_y, accel_z);

    // Read altitude data from BMP390
    altitude = bmp390.readAltitude();

    
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
    currentPressure = bmp390.readPressure()/100 //hectopascals
}
}
