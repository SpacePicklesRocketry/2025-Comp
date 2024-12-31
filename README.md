# **2024 Code Repository: Space Pickles**

Welcome to the official code repository for **Space Pickles**, a student-driven rocketry team based in **Fremont, California**, with members primarily from **American High School**.
![image](https://github.com/user-attachments/assets/37b6efee-77e0-403f-8f94-fd2e68476cfa)

---

## **Code Highlights**

### **Sensor Fusion of Gyro + Accelerometer**
We use a **Kalman Filter** to integrate data from the gyroscope and accelerometer, providing accurate orientation information. This reduces noise and drift for reliable angle calculations, even under dynamic conditions.

### **Localization Based on Sensor Fusion**
By combining sensor fusion outputs and velocity integration, our localization system estimates the rocket's position and trajectory in real time.

### **Timesync of Data Across Sensors**
Our system ensures synchronized data acquisition from multiple sensors, maintaining consistency in sensor readings by employing precise timestamps.

### **Redundant Sensors**
To enhance reliability, redundant sensors are incorporated into the system. If a sensor fails, the system dynamically switches to backup sensors for uninterrupted data logging.

### **Writing to Flash + SD Card**
Critical flight data is stored on both flash memory and an SD card for redundancy, ensuring data integrity and accessibility post-flight.

### **Simulation**
We use simulation frameworks to test the rocket's flight characteristics, sensor behavior, and code reliability in virtual environments before deployment.

### **Calibration of Sensors**
Comprehensive sensor calibration ensures accurate and reliable readings by eliminating offsets and biases in raw sensor data.

---

## **Setup Instructions**

**General**
   - Clone into your IDE, then open `main.ino` in the Arduino IDE.
   - **Important:** Ensure that `main.ino` is located in a folder named `main`. This is a requirement for the Arduino IDE.

**Serial Monitor:**
   - Always close the Serial Monitor before uploading code to the board. Leaving the Serial Monitor open can cause unexpected data leaks or interfere with the upload process.

**Board Visibility:**
   - If the board is not recognized by your computer, double-tap the reset button on the **MKR Zero** to reboot the board into bootloader mode.

---

This repository represents the collaborative efforts of the **Space Pickles** team. For questions or contributions, feel free to contact the team or submit a pull request.
