# Component Test Code

This folder contains test programs for the main components of the ESP32-S3 3D Scanner project.

The code is used to verify each component before integrating it into the main system.

## Components Tested

* ESP32-S3 GPIO
* A4988 Turntable Motor
* A4988 Z-axis Motor
* TF-Luna Distance Sensor
* Z-axis HOME Button
* Status LED

## Tools

* Arduino IDE
* PlatformIDE
* VSCode IDE
* ESP32-S3 N16R8

## Test Purpose

Each program checks the basic operation, wiring, GPIO assignment, and communication of its corresponding component.

After all tests are completed successfully, the verified code will be integrated into the main scanner firmware.
