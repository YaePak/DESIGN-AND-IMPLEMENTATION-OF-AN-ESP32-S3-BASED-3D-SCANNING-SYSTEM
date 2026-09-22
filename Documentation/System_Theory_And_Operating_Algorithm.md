# System Theory and Operating Algorithm

## 1. Operating Theory

The proposed system is designed as a **3D object scanning system** that collects distance measurements from an object's surface at different positions.

The **ESP32-S3** acts as the main controller of the system. It controls the scanning process, communicates with the TF-Luna distance sensor, and generates control signals for the stepper motor driver.

The **Stepper Motor**, controlled through the **A4988 driver**, rotates the object or scanning platform by a predefined angle. The motor provides precise and repeatable movement, allowing measurements to be taken at multiple positions around the object.

At each scanning position, the **TF-Luna ToF (Time-of-Flight) LiDAR sensor** measures the distance between the sensor and the object's surface. The measured distance is transmitted digitally to the ESP32-S3.

By combining the **distance measurement** with the corresponding **rotation angle**, the system can collect multiple points representing the surface of the object.

After completing one scanning cycle, the collected measurement data can be used for further processing and **3D reconstruction**, such as generating a **point cloud** of the scanned object.

---

## 2. System Operating Algorithm

The system operates according to the following sequence:

1. Initialize the ESP32-S3 and configure all required GPIO and communication interfaces.
2. Initialize the TF-Luna distance sensor.
3. Initialize the A4988 driver and stepper motor.
4. Set the scanning mechanism to its initial position.
5. Rotate the object or scanning platform by a predefined angular step.
6. Stop or stabilize the motor at the current position.
7. Measure the distance between the TF-Luna sensor and the object.
8. Store the measured distance together with the corresponding scanning angle.
9. Repeat the rotation and measurement process until a complete scanning cycle is finished.
10. Collect all measurement data for further processing.
11. Generate or transmit the collected data for **3D point cloud reconstruction**.

---

## 3. System Data Flow

```text
Stepper Motor Movement
          ↓
    Object Position
          ↓
TF-Luna Distance Measurement
          ↓
       ESP32-S3
          ↓
Store Distance + Angle Data
          ↓
     Repeat Scanning
          ↓
   Complete Scan Cycle
          ↓
   Point Cloud Generation
          ↓
   3D Object Reconstruction
```

## 4. Scanning Principle

The basic scanning principle is based on collecting distance information at multiple angular positions.

For each position:

* The stepper motor determines the scanning angle.
* The TF-Luna measures the distance to the object.
* The ESP32-S3 stores the measurement data.

By repeating this process around the object, multiple surface points can be collected. These points can later be processed to represent the three-dimensional shape of the scanned object.
