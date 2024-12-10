#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>

// Define thresholds
const float launch_accel_threshold = 75; // m/s^2
const float coast_accel_threshold = 40; // m/s^2
const float apogee_altitude_threshold = 240.792; // m


// Initialize sensors
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  Wire.begin();

  Serial.println("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Done.");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  if (!bmp.begin()) {
    Serial.println("Failed to find BMP085 sensor");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BMP085 Found!");
  }

// Function to read sensor data
void read_sensor_data(float &accel_x, float &accel_y, float &accel_z, float &altitude) {
    // Read acceleration data from MPU6050
    mpu.getAcceleration(&accel_x, &accel_y, &accel_z);

    // Read altitude data from BMP085
    const altitude = bmp.readAltitude();
}

// Main loop
void loop() {
    float accel_x, accel_y, accel_z, altitude;
    read_sensor_data(accel_x, accel_y, accel_z, altitude);
    float current_pressure = 1013;
    float last_pressure = 1013;


    float current_phase = "ground";
    // Detect flight phases based on sensor data and thresholds
    if (current_phase == "ground" && accel_z >= launch_accel_threshold) {
        Serial.println("Launch detected!");
        current_phase = "powered_ascent";
    } else if (current_phase == "powered_acsent" && accel_z > coast_accel_threshold && current_pressure-last_pressure < 1 ) {
        Serial.println("Coasting Phase detected!");
        current_phase = "coasting";
    } else if (current_phase == "coasting" && abs(altitude - apogee_altitude_threshold)<10 && accel_z < 5) {
        Serial.println("Apogee detected!");
        current_phase = "apogee";
    } else if (current_phase == "apogee" && 0 > accel_z ) {
        Serial.println("Descent detected!");
        current_phase = "descent";
    } else if (current_phase == "descent" && abs(0 - accel_z) < 2) { // <2 is accounting for hallucination data
        Serial.println("Landing detected!");
        current_phase = "landing";
    } else {
        Serial.println("PROBLEM DETECTED!");
    }

    lastPressure = currentPressure;
    currentPressure = bmp.readPressure()/100

    // Delay for sensor readings
    delay(100); 
}