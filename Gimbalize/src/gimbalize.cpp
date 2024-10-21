 #include <Wire.h>
 #include <Adafruit_MPU6050.h>
 #include <Adafruit_Sensor.h>
 #include <Servo.h>

int offset1 = 42;
int offset2 = 12;
const int maxAngle = 130;

Adafruit_MPU6050 mpu;
Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  servo1.attach(A6);
  servo2.attach(A7);

  servo1.write(offset1);
  servo2.write(offset2);
  Serial.println("Setup complete");
  delay(5000);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("Accel: ");
  Serial.println(a.acceleration.x);
  Serial.println("Gyro: ");
  Serial.println(g.gyro.x);
  Serial.println("Temp: ");
  Serial.println(temp.temperature);

  float tilt1 = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
  float tilt2 = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

  Serial.print("Tilt1: ");
  Serial.print(tilt1);
  Serial.print(", Tilt2: ");
  Serial.println(tilt2);

  controlFlaps(tilt1, tilt2);

  delay(100);
}

void controlFlaps(float tilt1, float tilt2) {
  int servo1Angle = constrain(map(tilt1, -90, 90, offset1, offset1 + maxAngle), offset1, maxAngle);
  int servo2Angle = constrain(map(tilt2, -90, 90, offset2, offset2 + maxAngle), offset2, maxAngle);

  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
}
