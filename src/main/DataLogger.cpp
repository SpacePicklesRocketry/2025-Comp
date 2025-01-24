#include "DataLogger.h"
#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN SDCARD_SS_PIN

File dataFile;
int test_number = 0; // Global test number for unique file names
String filename;

void initializeSDCard() {
    if (!SD.begin(SD_CS_PIN)) { 
        Serial.println("SD card initialization failed!");
        while (1) {
            delay(10); // Infinite loop to prevent further execution
        }
    }
    Serial.println("SD card initialized!");
}

void createLogFile() {
    char filename[20]; // Buffer for file name (max 8.3 format: "Int_XXX.txt")
    
    do {
        // Generate file name as "Int_<test_number>.txt"
        snprintf(filename, sizeof(filename), "Int_%d.csv", test_number);
        test_number++;
    } while (SD.exists(filename)); // Check if file exists

    // Debug: Print the final file name
    Serial.print("Creating file: ");
    Serial.println(filename);

    // Open the file for writing
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        // Write header row
        dataFile.println("Time (ms), Angle X (deg), Angle Y (deg), Angle Z (deg), Temperature (C), Pressure (Pa), Altitude (m)");
        dataFile.close();
        Serial.print("Log file created: ");
        Serial.println(filename);
    } else {
        Serial.print("Error creating file: ");
        Serial.println(filename);
    }
}
#include "DataLogger.h"
#include <SD.h>

extern File dataFile; // Ensure this is declared and properly initialized in your program

void logData(const SensorData& data) {
    dataFile = SD.open("data_log.csv", FILE_WRITE); // Open the log file
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
        dataFile.close();
    } else {
        Serial.println("Error opening file for writing!");
    }
}