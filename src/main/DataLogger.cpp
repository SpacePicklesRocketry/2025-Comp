#include "DataLogger.h"
#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN SDCARD_SS_PIN

File dataFile;
int test_number = 0;
char filename[20];    // TODO: CHECK BUFFER LENGTH

void initializeSDCard() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1) {
            delay(10); //infinite loop kills it :)
        }
    }
    Serial.println("SD card initialized!");
}

void createLogFile() {
    do {
        snprintf(filename, sizeof(filename), "LOG_%d.csv", test_number);
        test_number++;
    } while (SD.exists(filename));

    Serial.print("Creating file: ");
    Serial.println(filename);

    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        // Write header row 
        dataFile.println("Time, Roll, Pitch, Yaw, AccelX, AccelY, AccelZ, Altitude, Liftoff, Apogee");
        dataFile.close();
        Serial.print("Log file created: ");
        Serial.println(filename);
    } else {
        Serial.print("Error creating file: ");
        Serial.println(filename);
    }
}

void logData(const SensorData& data) {
    dataFile = SD.open(filename, FILE_WRITE);  // Use the dynamically generated filename
    if (dataFile) {
        dataFile.print(data.timestamp);
        dataFile.print(", ");
        dataFile.print(data.roll);
        dataFile.print(", ");
        dataFile.print(data.pitch);
        dataFile.print(", ");
        dataFile.print(data.yaw);
        dataFile.print(", ");
        dataFile.print(data.accelX);
        dataFile.print(", ");
        dataFile.print(data.accelY);
        dataFile.print(", ");
        dataFile.print(data.accelZ);
        dataFile.print(", ");
        dataFile.print(data.altitude);
        dataFile.print(", ");
        dataFile.print(data.liftoffDetected);
        dataFile.print(", ");
        dataFile.print(data.apogeeDetected);
        // dataFile.print(", ");
        // dataFile.print(airbrakeStatus.airbrakesDeployed);
        dataFile.println();
        dataFile.close();
    } else {
        Serial.println("Error opening file for writing!");
    }
}