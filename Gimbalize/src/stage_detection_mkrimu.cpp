#include <Wire.h>
#include <stage_detection.h>
#include <math.h>
#include <MKRIMU.h>
#include <Adafruit_BMP3XX.h>
// Set Starting Phase
current_stage = FlightPhase.GROUND;

Adafruit_BMP3XX bmp390;

#define MAIN_SENSOR_ADDR 0x77
#define BACKUP_SENSOR_ADDR 0x76

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

  // Check for the main BMP390 sensor
  if (bmp390.begin_I2C(MAIN_SENSOR_ADDR)) {
    Serial.println("Main BMP390 sensor found!");
    active_sensor_address = MAIN_SENSOR_ADDR;
    break;
  }
  // Check for the backup BMP390 sensor
  else if (bmp390.begin_I2C(BACKUP_SENSOR_ADDR)) {
    Serial.println("Backup BMP390 sensor found!");
    active_sensor_address = BACKUP_SENSOR_ADDR;
  } 
  // No sensor found
  else {
    Serial.println("Failed to find any BMP390 sensor!");
    while (1) {
      delay(10);
    }
  }
}

// Main loop
void detectFlightPhase_mkrimu(float accel_x, float accel_y, float accel_z, float altitude, float current_pressure, float last_pressure){

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

    // Delay for sensor readings
    delay(100); 
}
