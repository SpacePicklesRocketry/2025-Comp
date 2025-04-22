#include "Sensors.h"
#include <MKRIMU.h>
#include <Arduino_MKRGPS.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <SimpleKalmanFilter.h>


#define BMP1_ADDR 0x76
#define BMP2_ADDR 0x77
#define SEALEVELPRESSURE_HPA 1019
#define APOGEE_DETECTION_THRESHOLD -0.5
#define MOVING_AVG_WINDOW 10
#define LIFTOFF_ACCELERATION_THRESHOLD 3.0  // <- Add this if missing from Sensors.h
#define SPEED_MOVING_AVG_WINDOW 6  // Define the size of the moving average window


Adafruit_BMP3XX bmp1;
Adafruit_BMP3XX bmp2;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const float earthRadius = 6371.0; // in km

// Globals
float previousAltitude = 0;
float baseAltitude = 0;
float initialRoll = 0, initialPitch = 0, initialYaw = 0;
unsigned long initialTime;
float refLat = 0.0;
float refLon = 0.0;
bool gpsRefSet = false;



SimpleKalmanFilter kalmanAX(5, 0.6, 0.25);
SimpleKalmanFilter kalmanAY(5, 0.8, 0.8);
SimpleKalmanFilter kalmanAZ(1, .1, 0.4);



// Moving average buffer for altitude
float altitudeHistory[MOVING_AVG_WINDOW] = {0};
int altitudeIndex = 0;
bool bufferFilled = false;

// 3D position struct
struct position2 {
    float x, y, z;
};


//function prototypes
float computeMovingAverage(float newAltitude);
double blend(double a, double b, double weightA, double weightB);
position2 lonLatToEuclidian(double lon, double lat, double refLat, double refLon);
float computeSpeedMovingAverage(float newSpeed);
void detectApogee(SensorData &data, const SensorData &prev);
void detectLiftoff(SensorData &data, const SensorData &prev, float filteredAccelZ);

// Calibration offsets
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;
float accelOffsetX = 0.0, accelOffsetY = 0.0, accelOffsetZ = 0.0;


//deadband
float applyDeadband(float value, float threshold) {
    return (fabs(value) < threshold) ? 0.0 : value;
}


void calibrateSensors() {
    Serial.println("Sensors Calibrating");
    const int sampleCount = 100; // Number of samples to average
    float gyroSumX = 0.0, gyroSumY = 0.0, gyroSumZ = 0.0;
    float accelSumX = 0.0, accelSumY = 0.0, accelSumZ = 0.0;

    // Collect sensor readings for calibration
    for (int i = 0; i < sampleCount; i++) {
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

        gyroSumX += gyro.x();
        gyroSumY += gyro.y();
        gyroSumZ += gyro.z();
        accelSumX += accel.x();
        accelSumY += accel.y();
        accelSumZ += accel.z();

        
        delay(10); // short delay between samples
    }

    gyroOffsetX = gyroSumX / sampleCount;
    gyroOffsetY = gyroSumY / sampleCount;
    gyroOffsetZ = gyroSumZ / sampleCount;
    accelOffsetX = accelSumX / sampleCount;
    accelOffsetY = accelSumY / sampleCount;
    accelOffsetZ = accelSumZ / sampleCount;
    Serial.println("Sensors Calibrated");
}

void setReferencePoint(float lat, float lon) {
    refLat = lat;
    refLon = lon;
    gpsRefSet = true;
}


void initializeSensors() {
    Serial.begin(9600);
    
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055");
        while (1) delay(10);
    }
    Serial.println("BNO055 Initialized");

    if (!GPS.begin()) {
        Serial.println("Failed to initialize GPS!");
        while (1);
    }

    delay(3000);  // Give GPS some time to start acquiring

    while (!GPS.available()) {
        Serial.println("Waiting for GPS fix...");
        delay(1);
    }

    Serial.print("GPS initialized with ");
    Serial.print(GPS.satellites());
    Serial.println(" satellites");

    if (!bmp1.begin_I2C(BMP1_ADDR)) {
        Serial.println("Sensor 1 not found at address 0x76");
    } else {
        bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
        Serial.println("Sensor 1 initialized at address 0x76");
    }

    if (!bmp2.begin_I2C(BMP2_ADDR)) {
        Serial.println("Sensor 2 not found at address 0x77");
    } else {
        bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
        Serial.println("Sensor 2 initialized at address 0x77");
    }

    delay(2000); // Allow sensors to stabilize

    // Discard first few readings
    for (int i = 0; i < 5; i++) {
        float tmp = readAltitudeFromBMP();
        Serial.print("Discarding altitude sample: ");
        Serial.println(tmp, 2);
        delay(200);
    }

    baseAltitude = readAltitudeFromBMP();
    Serial.print("Base altitude set to: ");
    Serial.println(baseAltitude, 2);

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    initialRoll = euler.x();
    initialPitch = euler.y();
    initialYaw = euler.z();


    setReferencePoint(GPS.latitude(), GPS.longitude());


    initialTime = millis();
    Serial.println("Initialization complete.");
}

SensorData readSensors(float deltaTime, SensorData &prev) {
    SensorData data;
    unsigned long now = millis();
    
    // Orientation (relative)
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    data.roll = euler.x() - initialRoll;
    data.pitch = euler.y() - initialPitch;
    data.yaw = euler.z() - initialYaw;

    // Gyroscope + Linear Accel with Kalman and Offsets
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    data.gyroX = gyro.x() - gyroOffsetX;
    data.gyroY = gyro.y() - gyroOffsetY;
    data.gyroZ = gyro.z() - gyroOffsetZ;

    float tempAccelX = kalmanAX.updateEstimate(accel.x() - accelOffsetX);
    float tempAccelY = kalmanAY.updateEstimate(accel.y() - accelOffsetY);
    float tempAccelZ = kalmanAZ.updateEstimate(accel.z() - accelOffsetZ);


    // Apply deadband
    float filtAX = applyDeadband(tempAccelX, 0.08);
    float filtAY = applyDeadband(tempAccelY, 0.09);
    float filtAZ = applyDeadband(tempAccelZ, 0.08);

    data.accelX= filtAX;
    data.accelY= filtAY;
    data.accelZ= filtAZ;

    // Integrate for velocity and apply smoothing
    float velX = prev.velocityX + data.accelX * deltaTime;
    float velY = prev.velocityY + data.accelY * deltaTime;
    float velZ = prev.velocityZ + data.accelZ * deltaTime;

    data.velocityX = computeSpeedMovingAverage(velX);
    data.velocityY = computeSpeedMovingAverage(velY);
    data.velocityZ = computeSpeedMovingAverage(velZ);

    // Position integration
    data.positionX = prev.positionX + data.velocityX * deltaTime;
    data.positionY = prev.positionY + data.velocityY * deltaTime;
    data.positionZ = prev.positionZ + data.velocityZ * deltaTime;

    // Altitude and Rate of Change
    float rawAlt = blend(GPS.altitude(), readAltitudeFromBMP(), 0.85, 0.15);
    data.altitude = rawAlt - baseAltitude;
    data.rateOfChange = (data.altitude - previousAltitude) / deltaTime;
    previousAltitude = data.altitude;

    // GPS
    data.longitude = GPS.longitude();
    data.latitude = GPS.latitude();
    data.satellites = GPS.satellites();

    detectLiftoff(data, prev, filtAZ);
    detectApogee(data, prev);

    data.timestamp = now - initialTime;
    return data;
}



float readAltitudeFromBMP() {
    if (bmp1.performReading()) {
        return bmp1.readAltitude(SEALEVELPRESSURE_HPA);
    } else if (bmp2.performReading()) {
        return bmp2.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
        return 0;
    }
}

float computeMovingAverage(float newAltitude) {
    altitudeHistory[altitudeIndex] = newAltitude;
    altitudeIndex = (altitudeIndex + 1) % MOVING_AVG_WINDOW;

    if (!bufferFilled && altitudeIndex == 0) {
        bufferFilled = true;
    }

    float sum = 0;
    int count = bufferFilled ? MOVING_AVG_WINDOW : altitudeIndex;
    for (int i = 0; i < count; i++) {
        sum += altitudeHistory[i];
    }
    return sum / count;
}


float speedHistory[SPEED_MOVING_AVG_WINDOW];  // Array to store speed values
int speedIndex = 0;
bool speedBufferFilled = false;

float computeSpeedMovingAverage(float newSpeed) {
    // Add the new speed value to the history array
    speedHistory[speedIndex] = newSpeed;
    speedIndex = (speedIndex + 1) % SPEED_MOVING_AVG_WINDOW;

    // If the buffer isn't full yet, mark it as filled when we reach the first full cycle
    if (!speedBufferFilled && speedIndex == 0) {
        speedBufferFilled = true;
    }

    // Compute the sum of the values in the history array
    float sum = 0;
    int count = speedBufferFilled ? SPEED_MOVING_AVG_WINDOW : speedIndex;
    for (int i = 0; i < count; i++) {
        sum += speedHistory[i];
    }

    // Return the average speed
    return sum / count;
}


position2 lonLatToEuclidian(double lon, double lat, double refLat, double refLon) {
    double latRad = lat * M_PI / 180;
    double lonRad = lon * M_PI / 180;
    double refLatRad = refLat * M_PI / 180;
    double refLonRad = refLon * M_PI / 180;

    double x = earthRadius * (lonRad - refLonRad) * cos((latRad + refLatRad) / 2.0);
    double y = earthRadius * (latRad - refLatRad);
    return {x, y, 0};
}

double blend(double a, double b, double weightA, double weightB) {
    double totalWeight = weightA + weightB;
    return (weightA * a + weightB * b) / totalWeight;
}

void detectLiftoff(SensorData &data, const SensorData &prev, float filteredAccelZ) {
    if (!prev.liftoffDetected && filteredAccelZ > LIFTOFF_ACCELERATION_THRESHOLD) {
        data.liftoffDetected = true;
        data.liftoffTime = millis();
        Serial.print("Liftoff detected at: ");
        Serial.println(data.liftoffTime);
    } else {
        data.liftoffDetected = prev.liftoffDetected;
        data.liftoffTime = prev.liftoffTime;
    }
}

void detectApogee(SensorData &data, const SensorData &prev) {
    if (data.liftoffDetected && !prev.apogeeDetected && data.rateOfChange < APOGEE_DETECTION_THRESHOLD) {
        data.apogeeDetected = true;
        data.apogeeTime = millis();
        Serial.print("Apogee detected at: ");
        Serial.println(data.apogeeTime);
    } else {
        data.apogeeDetected = prev.apogeeDetected;
        data.apogeeTime = prev.apogeeTime;
    }
}
