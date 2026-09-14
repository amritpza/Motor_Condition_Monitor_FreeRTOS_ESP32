## Project Summary
This project implements a maintenance motor monitor using an ESP32-DevKitC V4. The firmware continuously samples high-frequency vibration and temperature data from an MPU6050 IMU bolted directly to a motor housing. Using real-time metrics, the device assesses motor health states, handles localized issues, maps thresholds, and communicates diagnostics concurrently.

## Hardware Architecture

![Block Diagram](images/Motor_Condition_Monitor.PNG)