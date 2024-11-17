#include <Wire.h>
#include "ADAFRUIT_BNO08X.h"
#include <RohmMultiSensor.h>


// Create sensor objects
BNO08x bno;
BD1020HFV sensorTmp;
// the above is equivalent to
// BD1020HFV sensorTmp(ANALOG_1);
// if you have the sensor connected to ANALOG_2, use the following
// BD1020HFV sensorTmp(ANALOG_2);

//Store gyro data
float x;
float y;
float z;

// Variables to store integrated angles
float angleX = 0;
float angleY = 0;
float angleZ = 0; 

//GYRO SETUP
// Define as -1 to disable these features.
#define BNO08X_INT  A4
//#define BNO08X_INT  -1
#define BNO08X_RST  A5
//#define BNO08X_RST  -1

#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed


// Time tracking
unsigned long lastTime;
unsigned long currentTime;

void setup() {
  // Initialize serial communication
  Serial.begin(9600); //Can change to 115200 if needed and supported
  while (!Serial) {
    delay(10); // Wait for serial monitor to open
  }
//Start I2C Communication 
 Wire.begin();

 // Scan for I2C devices
  Serial.println("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Done.");

  //if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO086 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO086 found!");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();
 
 
  if (!sensorTmp.init()) {
    Serial.println("Failed to find BD1020HFV chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BD1020HFV Found!");
 
 
  // initialize the Temp sensor with default settings
  Serial.println("t[dg C]");

  //Initialize time tracking
  lastTime = millis();
}

//Setup Confirmation for Gyro
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableGyro() == true) {
    Serial.println(F("Gyro enabled"));
    Serial.println(F("Output in form x, y, z, in radians per second"));
  } else {
    Serial.println("Could not enable gyro");
  }
}

void loop() {
  void loop() {
  // Get current time
  currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;

  // Was Gyro reset?
  if (bno.wasReset()) {
    Serial.print("Sensor was reset!");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (bno.getSensorEvent() == true) {

    // is it the correct sensor data we want?
    if (bno.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {

      x = bno.getGyroX();
      y = bno.getGyroY();
      z = bno.getGyroZ();

      // Integrate gyroscope data to get angles
      angleX += g.gyro.x * deltaTime * 180.0 / PI;
      angleY += g.gyro.y * deltaTime * 180.0 / PI;
      angleZ += g.gyro.z * deltaTime * 180.0 / PI;

      // Print angles
      Serial.print("Angle X: ");
      Serial.print(angleX);
      Serial.print(", Angle Y: ");
      Serial.print(angleY);
      Serial.print(", Angle Z: ");
      Serial.println(angleZ);
    }
  }
  
 
  //Read Temp Sensor Data
  sensorTmp.measure();
  float temperature = sensorTmp.temp;

  //Print Temp Sensor Data 
  Serial.print("Temperature: ");
  Serial.print(temperature);
  
}