# Remote-Three-Phase-Induction-Motor-Control
A remote control system for a three-phase based induction motor, utilizing an Arduino Nano for motor control and an Arduino Uno for display. The system uses PWM signals to drive MOSFET drivers, generating the necessary three-phase signals to control the motor. This project is meant to simulate a simple 3-phase motor open loop control system for an industrial application.

![image](https://github.com/user-attachments/assets/d5c52782-f168-446b-92df-fd759332b3c0)


## Features
- PWM Control: Generates three-phase signals by driving MOSFETs with PWM outputs from the Arduino Nano.
- Parameter Display: Real-time motor parameters, such as speed and current, are shown on an LCD connected to the Arduino Uno.
- Basic Remote Operation: Start/stop and speed adjustments controlled through a simple interface.

## Hardware Requirements
- Three-phase induction motor
- MOSFET drivers and inverter MOSFETs for three-phase signal generation
- Arduino Nano (for motor control)
- Arduino Uno (for display)
- LCD display for real-time monitoring
- Passive electronic elements (Resistors, Capacitors and Inductors)
- Optional: 12VDC power or grid supply

## Software
- Arduino IDE for firmware development
- Proteus Professional for simulating the circuit and code using VSIM
- C++ code to generate PWM signals on the Nano and display metrics on the Uno

## Getting Started
1. Clone this repository and upload the firmware to the Arduino Nano and Arduino Uno.
2. Set up the MOSFET drivers and connect the motor, display, and control circuits.
3. Power the system and use the interface for basic motor control and monitoring.

## Future Development (Rev B)
- ESP32 as main controller for display and motor control
- Network capability for remote monitoring and control
- Closed-loop feedback control using hall effect sensors, encoders and etc
- Improved fault detection and protection features
- Potential integration of PID control for more refined motor speed handling

![image](https://github.com/user-attachments/assets/2b13f1e8-8d4e-4567-9549-e1301dcd83d4)

