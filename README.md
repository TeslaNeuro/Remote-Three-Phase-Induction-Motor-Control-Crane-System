# Remote-Three-Phase-Inductor-Motor-Control
This repository contains Rev A of a remote control system for a three-phase induction motor, utilizing an Arduino Nano for motor control and an Arduino Uno for display. The system uses PWM signals to drive MOSFET drivers, generating the necessary three-phase signals to control the motor.

## Features
- PWM Control: Generates three-phase signals by driving MOSFETs with PWM outputs from the Arduino Nano.
- Parameter Display: Real-time motor parameters, such as speed and current, are shown on an LCD connected to the Arduino Uno.
- Basic Remote Operation: Start/stop and speed adjustments controlled through a simple interface.

## Hardware Requirements
Three-phase induction motor
MOSFET drivers and inverter MOSFETs for three-phase signal generation
Arduino Nano (for motor control)
Arduino Uno (for display)
LCD display for real-time monitoring

## Software
- Arduino IDE for firmware development
- C++ code to generate PWM signals on the Nano and display metrics on the Uno

## Getting Started
Clone this repository and upload the firmware to the Arduino Nano and Arduino Uno.
Set up the MOSFET drivers and connect the motor, display, and control circuits.
Power the system and use the interface for basic motor control and monitoring.

## Future Development (Rev B)
Network capability for remote monitoring and control
Improved fault detection and protection features
Potential integration of PID control for more refined motor speed handling
