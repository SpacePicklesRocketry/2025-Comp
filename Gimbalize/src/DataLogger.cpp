#include "DataLogger.h"
#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN SDCARD_SS_PIN

File dataFile;

void initializeSDCard() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("SD card initialized!");
}

void createLogFile() {
  dataFile = SD.open("data_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time (ms), Angle X (deg), Angle Y (deg), Angle Z (deg), Temperature (C), Pressure (Pa), Altitude (m)");
    dataFile.close();
  } else {
    Serial.println("Error opening file!");
  }
}

void logData(const SensorData& data) {
  dataFile = SD.open("data_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(data.timestamp);
    dataFile.print(", ");
    dataFile.print(data.angleX);
    dataFile.print(", ");
    dataFile.print(data.angleY);
    dataFile.print(", ");
    dataFile.print(data.angleZ);
    dataFile.print(", ");
    dataFile.print(data.temperature);
    dataFile.print(", ");
    dataFile.print(data.pressure);
    dataFile.print(", ");
    dataFile.println(data.altitude);
    dataFile.close();
  } else {
    Serial.println("Error opening file for writing!");
  }
}