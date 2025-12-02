# RC Robot with Autonomous Navigation  
A multifunctional Arduino-based robot supporting both manual (Bluetooth) control and autonomous obstacle-avoidance operation.  
The robot features:  
- L293D Motor Shield (AFMotor library)  
- HC-05/HC-06 Bluetooth control  
- Ultrasonic distance measurement  
- Servo-based scanning  
- I2C LCD with custom eye animations  
- Adaptive motor speed control  
- Path memory and stuck-detection logic  

---

## Features

### Manual Mode (Bluetooth)
- Control commands via Bluetooth:  
  - **F** – Move Forward  
  - **B** – Move Backward  
  - **L** – Turn Left  
  - **R** – Turn Right  
  - **S** – Stop  
  - **T** – Toggle Autonomous Mode  
  - **1–9** – Set speed from 10% to 90%  

- Receives acknowledgement for every command  
- Automatically returns to autonomous mode if no command is received for a set timeout  

---

## Autonomous Navigation Mode
- Uses ultrasonic sensor to detect obstacles  
- Adaptive speed control based on distance  
- Smart scanning using servo at five predefined angles  
- Chooses best direction (left, right, or straight)  
- Reverses if no clear path is found  
- Includes stuck-detection using a small 5-entry path memory  
- Random escape movement when stuck  

---

## Hardware Components

| Component | Description |
|----------|-------------|
| Arduino Uno / Mega | Main controller |
| L293D Motor Shield (Adafruit AFMotor) | Drives 4 DC motors |
| 4x DC Motors | Robot wheels |
| HC-SR04 Ultrasonic Sensor | Distance measurement |
| SG90 / MG90 Servo | Rotates ultrasonic sensor for scanning |
| HC-05 / HC-06 Bluetooth Module | Bluetooth communication |
| I2C 16x2 LCD | Display messages and eye animation |
| Battery Pack | Power supply |

---

## Pin Configuration

| Function | Pin |
|----------|-----|
| Bluetooth RX/TX | 2, 3 |
| Ultrasonic Trigger | 9 |
| Ultrasonic Echo | 10 |
| Servo Motor | 11 |
| Motor Ports | Motor1 (M1), Motor2 (M2), Motor3 (M3), Motor4 (M4) on shield |

---

## Code Structure

### Main Components  
- **Motor Control**  
  - Ramp-up and ramp-down speed transitions  
  - Direction logic for all four motors  
- **Bluetooth Control Handler**  
  - Interprets commands  
  - Handles manual override logic  
- **Autonomous Navigation**  
  - Scanning routine  
  - Decision-making based on sensor data  
  - Path memory and stuck detection  
- **Sensor Processing**  
  - Median filtering for accurate ultrasonic readings  
- **LCD User Interface**  
  - Status messages  
  - Animated blinking eyes  
- **Startup Motor Test**  
  - Rigorous motor and direction validation  

---

## Commands Summary

| Command | Action |
|---------|--------|
| F | Forward |
| B | Backward |
| L | Turn Left |
| R | Turn Right |
| S | Stop |
| T | Toggle Autonomous Mode |
| 1–9 | Set speed level |

---

## Autonomous Logic Overview

1. Check distance ahead  
2. If distance is:  
   - **Safe** → Move forward with adaptive speed  
   - **Obstacle** → Scan left, center, right  
   - **Critical** → Stop and reverse  
3. Determine best direction from scan results  
4. Move accordingly  
5. If repeating patterns indicate being stuck → Execute escape maneuver  

---

## Display System

### LCD States
- Boot messages  
- Current action (Forward, Backward, Obstacle, Too Close)  
- Distance and speed in autonomous mode  
- Blinking eyes animation when idle  

---

## Motor Test Routine
Automatically runs during startup:
- Tests each motor individually  
- Displays motor status on LCD  
- Sends status messages over Bluetooth  

---

## Setup Instructions

1. Install required libraries:  
   - AFMotor  
   - LiquidCrystal_I2C  
   - SoftwareSerial  
   - Servo  

2. Assemble all components according to the wiring table.  
3. Upload the provided code to your Arduino.  
4. Power the robot and connect to Bluetooth using a mobile app or controller.  
5. Test motor directions using the automatic motor test.  
6. Operate in manual or autonomous mode.

---

## File Description

- **Main .ino File** – Contains full project code including motor control, Bluetooth handler, autonomous system, sensor logic, and LCD interface.  
- **README.md** – Documentation explaining robot behavior and system architecture.

---

## Future Improvements

- Add PID control for smoother steering  
- Implement mapping or SLAM  
- Add infrared or LIDAR sensors  
- Battery monitoring system  
- Improved stuck recovery logic

---

## License

This project is open for modification, personal use, and educational purposes.

