# Project Weekly Progress Roadmap

## Project Title

**DESIGN AND IMPLEMENTATION OF AN ESP32-S3-BASED 3D OBJECT SCANNING SYSTEM**

This folder contains the weekly progress reports of the project.

The reports are organized according to the actual development progress of the team. Each week documents the completed tasks, individual responsibilities, current results, problems encountered, and the plan for the following week.

---

## Development Roadmap

### Week 1 – Research and Initial Design

- Research ESP32-S3
- Research Stepper Motor + A4988
- Research TF-Luna
- Develop the system operating principle and algorithm.
- Create the initial 3D mechanical design using Shapr3D.

---

### Week 2 – Hardware Setup and Sensor Testing

- Assemble hardware components
- Set up ESP32-S3 development environment
- Test TF-Luna sensor
- Test Stepper Motor + A4988
- Perform basic calibration and component testing

---

### Week 3 – Motion Control and System Integration

- Verify measurement accuracy and stability.
- Evaluate the sensor performance for 3D scanning.
- Integrate the sensor and motion control system.
- Implement the basic scanning sequence.
- Perform initial scanning tests.

---

### Week 4 – Data Collection and File Generation

- Collect distance and position data.
- Convert the scanning data into coordinate points.
- Generate and store the scanning data file in the ESP32-S3.
- Perform initial scanning tests.

---

### Week 5 – Wi-Fi File Transfer and Data Processing

- Implement Wi-Fi communication.
- Transfer the generated scanning file from the ESP32-S3 to a computer.
- Develop or test the PC-side data receiving process.
- Verify file integrity and scanning data.

---

### Week 6 – 3D Reconstruction

- Scan more complex objects.
- Process the collected point cloud.
- Apply noise filtering and outlier removal.
- Generate the 3D model and evaluate its accuracy.

---

### Week 7 – Final Report and System Testing

- Complete the final technical report.
- Perform complete system testing.
- Verify the overall system performance.
- Prepare the project for final presentation.

---


---

# Team Responsibilities

The project tasks are divided among three main members:

| Member | Student Number | Main Responsibilities |
|---|---|---|
| **Khoa** | 24119048 | Project planning, mechanical assembly, 3D printing, motion testing, Wi-Fi streaming, and 3D reconstruction |
| **Tài** | 24119080 | Electronic components, sensors, ESP32-S3 testing, data processing, filtering, and software documentation |
| **Tuấn** | 24119096 | System architecture, ESP32-S3, motor control, algorithms, and technical reporting | 

---

# Development Phases

The project is divided into the following main phases:

```text
Project Preparation
        ↓
Research and System Assembly
        ↓
Sensor Setup and Calibration
        ↓
Motion Control
        ↓
Sequential Scanning
        ↓
FreeRTOS Multi-Tasking
        ↓
Data Streaming
        ↓
Optimization
        ↓
3D Reconstruction
        ↓
System Validation and Final Report