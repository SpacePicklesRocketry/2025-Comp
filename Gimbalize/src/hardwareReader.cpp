#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>

// Create sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

// Variables to store integrated angles
float angleX = 0;
float angleY = 0;
float angleZ = 0;

// Time tracking
unsigned long lastTime;
unsigned long currentTime;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // Wait for serial monitor to open
  }

  // Initialize I2C communication
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

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BMP180 Found!");

  // Initialize time tracking
  lastTime = millis();
}

void loop() {
  // Get current time
  currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;

  // Get MPU6050 events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

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

  // Read BMP180 data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(1013.25); // Adjust sea level pressure if needed

  // Print BMP180 data
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" *C, Pressure: ");
  Serial.print(pressure);
  Serial.print(" Pa, Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");

  // Delay between readings
  delay(100);
}
