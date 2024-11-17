#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <SD.h>

void setup() {
  // Initialize pins as outputs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    
    //RED
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    analogWrite(LEDR, 0);

    delay(1000);

    //RED
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    analogWrite(LEDR,255);

    delay(1000);

    while (1);
  }
  Serial.print("IMU initialized!");
  
}

void loop() {

  Bool sd_init = false;
  if (!SD.begin(4)) { //REPLACE WITH SD CARD PIN NUMBER
    Serial.println("SD initialization failed!");
    sd_init = false;

    //RED
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    analogWrite(LEDR, 0);

    delay(1000);

    //RED
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    analogWrite(LEDR,255);

    delay(1000);

    while (1);
  else {
    sd_init = true;
  }
  }

  int test_number = 1;

  if (sd_init = true && SD.exists("test_" + test_number + ".txt")) {
    
    test_number = test_number++;
    myFile = SD.open("test_" + test_number + ".txt", FILE_WRITE);
  }
  Serial.print("SD card initialized!"); 

  myFile.println("Gyro_X ","Gyro_Y", "Gyro_Z", "Acc_X","Acc_Y" ,"Acc_Z", "Mag_X", "Mag_Y", "Mag_Z");

  float gx, gy, gz;
  float ax, ay, az;
  float mx, my, mz;

for(int i = 0; i < 400; i++){

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    myFile.print(gx + ", " +  gy + ", " + gz);


    // Serial.print(gx);
    // Serial.print('\t');
    // Serial.print(gy);
    // Serial.print('\t');
    // Serial.println(gz);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    myFile.print(ax + ", " +  ay + ", " + az);

    // Serial.print(ax);
    // Serial.print('\t');
    // Serial.print(ay);
    // Serial.print('\t');
    // Serial.println(az);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    myFile.print(mx + ", " +  my + ", " + mz + "\n");

    // Serial.print(mx);
    // Serial.print('\t');
    // Serial.print(my);
    // Serial.print('\t');
    // Serial.println(mz);
  }
  delay(100);
}
 myFile.close()

 //Green
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
    analogWrite(LEDR, 0);

    delay(5000);

    //green
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    analogWrite(LEDR,255);

    delay(1000);
}