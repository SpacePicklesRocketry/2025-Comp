#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SimpleKalmanFilter.h>

#define BMP1_ADDR 0x76 // Address of first BMP390
#define BMP2_ADDR 0x77 // Address of second BMP390

Adafruit_BMP3XX bmp1; // First sensor instance
Adafruit_BMP3XX bmp2; // Second sensor instance

#define LED_PIN LED_BUILTIN
#define SEALEVELPRESSURE_HPA 1013.25 // Standard sea-level pressure in hPa

float previousAltitude = 0;
float currentAltitude = 0;
float apogeeThreshold = -.5; // Apogee detection: Negative rate of climb

SimpleKalmanFilter altitudeKalman(1, 1, 0.01); // Kalman filter instance: processNoise, measurementNoise, estimateError

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially

  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial monitor to open

  Serial.println("Initializing BMP390 sensors...");

  if (!bmp1.begin_I2C(BMP1_ADDR)) {
    Serial.println("Sensor 1 not found at address 0x76");
  } else {
    Serial.println("Sensor 1 initialized at address 0x76");
  }

  if (!bmp2.begin_I2C(BMP2_ADDR)) {
    Serial.println("Sensor 2 not found at address 0x77");
  } else {
    Serial.println("Sensor 2 initialized at address 0x77");
  }

  if (!bmp1.begin_I2C(BMP1_ADDR) && !bmp2.begin_I2C(BMP2_ADDR)) {
    Serial.println("Both sensors failed to initialize. Check wiring.");
    while (1);
  }

  // Set up sensor sampling and configuration (same for both sensors)
  if (bmp1.begin_I2C(BMP1_ADDR)) {
    bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  if (bmp2.begin_I2C(BMP2_ADDR)) {
    bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  Serial.println("Initialization complete.");
}

void loop() {
  bool bmp1Available = bmp1.begin_I2C(BMP1_ADDR);
  bool bmp2Available = bmp2.begin_I2C(BMP2_ADDR);

  if (bmp1Available || bmp2Available) {
    if (bmp1Available && bmp1.performReading()) {
      currentAltitude = bmp1.readAltitude(SEALEVELPRESSURE_HPA);
    } else if (bmp2Available && bmp2.performReading()) {
      currentAltitude = bmp2.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      Serial.println("Both sensors failed to read.");
      delay(100);
      return;
    }

    // Apply Kalman filter to smooth the altitude data
    float filteredAltitude = altitudeKalman.updateEstimate(currentAltitude);
    float rateOfClimb = filteredAltitude - previousAltitude;

    if (rateOfClimb < apogeeThreshold) {
      Serial.println("Apogee detected! Turning on LED.");
      digitalWrite(LED_PIN, HIGH);
      delay(500); // Leave LED on for half a second
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    Serial.print("Current Altitude: ");
    Serial.println(filteredAltitude);
    Serial.print("Rate of Climb: ");
    Serial.println(rateOfClimb);

    previousAltitude = filteredAltitude;
  } else {
    Serial.println("No sensors available.");
  }

  delay(50); // Loop delay for stability
}