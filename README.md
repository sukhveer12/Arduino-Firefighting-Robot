# Arduino Firefighting Robot
## Description
This is the code for an Arduino Firefighting Robot that autonomously searches for a fire in a maze and blows it out.

## Installation
Building this project requires the following hardware:
* 1 x Arduino Uno
* 2 x Servo motors
* 1 x HC-SR04 ultrasonic sensor
* 1 x Flame Sensor
* 1 x Fan (may require external power source)

Pin | Input/Output | Function
--- | ------------ | ---------
Analog Pin A0 | Input | Flame sensor
Digital Pin 2 | Output | Ultrasonic sensor TRIG
Digital Pin 3 | Input | Ultrasonic sensor ECHO
Digital Pin 6 | Output | Fan Control
Digital Pin 8 | Output | Left Servo Control
Digital Pin 9 | Output | Right Servo Control

The code can be compiled and download to the Arduino Uno board using the (Arduino IDE)[https://www.arduino.cc/en/main/software].
