#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("I2C Scanner Initialized");
  Wire.begin();
}

void loop() {
  Serial.println("Scanning I2C bus...");

  int devicesFound = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
      devicesFound++;
    }
  }

  if (devicesFound == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Number of I2C devices found: ");
    Serial.println(devicesFound);
  }

  delay(200);
}