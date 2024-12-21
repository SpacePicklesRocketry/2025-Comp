#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <FlashStorage.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

FlashStorage(calibDataStorage, adafruit_bno055_offsets_t);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055! Check wiring.");
    while (1);
  }

  Serial.println("BNO055 initialized!");

  if (loadCalibration()) {
    Serial.println("Calibration data loaded from flash storage.");
  } else {
    Serial.println("No valid calibration data found. Perform manual calibration.");
  }

  Serial.println("Commands:");
  Serial.println("'s' - Save calibration data to flash");
  Serial.println("'r' - Reset calibration data");
}

void loop() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("System Calibration: ");
  Serial.print(system);
  Serial.print(", Gyro: ");
  Serial.print(gyro);
  Serial.print(", Accel: ");
  Serial.print(accel);
  Serial.print(", Mag: ");
  Serial.println(mag);

  if (bno.isFullyCalibrated()) {
    Serial.println("Saving calibration data to flash storage...");
    saveCalibration();
    delay(5000);
  }

  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      saveCalibration();
    } else if (command == 'r') {
      resetCalibration();
    }
  }

  delay(1000);
}

void saveCalibration() {
  adafruit_bno055_offsets_t calibrationData;
  bno.getSensorOffsets(calibrationData);

  calibDataStorage.write(calibrationData);

  Serial.println("Calibration data saved to flash storage:");
  displayOffsets(calibrationData);
}

bool loadCalibration() {
  adafruit_bno055_offsets_t calibrationData = calibDataStorage.read();

  if (calibrationData.accel_offset_x == 0 && calibrationData.accel_offset_y == 0 &&
      calibrationData.accel_offset_z == 0) {
    return false;
  }

  bno.setSensorOffsets(calibrationData);

  Serial.println("Loaded calibration data:");
  displayOffsets(calibrationData);
  return true;
}

void resetCalibration() {
  adafruit_bno055_offsets_t emptyOffsets = {};
  calibDataStorage.write(emptyOffsets);

  Serial.println("Calibration data reset. Perform manual calibration to save new data.");
}

void displayOffsets(const adafruit_bno055_offsets_t& offsets) {
  Serial.println("Calibration Offsets:");
  Serial.print("Accel Offset X: "); Serial.println(offsets.accel_offset_x);
  Serial.print("Accel Offset Y: "); Serial.println(offsets.accel_offset_y);
  Serial.print("Accel Offset Z: "); Serial.println(offsets.accel_offset_z);

  Serial.print("Gyro Offset X: "); Serial.println(offsets.gyro_offset_x);
  Serial.print("Gyro Offset Y: "); Serial.println(offsets.gyro_offset_y);
  Serial.print("Gyro Offset Z: "); Serial.println(offsets.gyro_offset_z);

  Serial.print("Mag Offset X: "); Serial.println(offsets.mag_offset_x);
  Serial.print("Mag Offset Y: "); Serial.println(offsets.mag_offset_y);
  Serial.print("Mag Offset Z: "); Serial.println(offsets.mag_offset_z);

  Serial.print("Accel Radius: "); Serial.println(offsets.accel_radius);
  Serial.print("Mag Radius: "); Serial.println(offsets.mag_radius);
}