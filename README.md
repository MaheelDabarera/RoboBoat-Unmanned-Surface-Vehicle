# RoboBoat: Unmanned Surface Vehicle

This project demonstrates autonomous navigation for a boat using GPS and compass. The boat follows a series of waypoints specified by their latitude and longitude coordinates. It uses PID control to adjust motor speeds and heading angle to navigate towards the waypoints.

## Table of Contents

- [Introduction](#introduction)
- [Requirements](#requirements)
- [Setup](#setup)
- [Usage](#usage)
- [Considerations for Using on Arduino](#considerations-for-using-on-arduino)
- [Google Scholar Profile](#google-scholar-profile)
- [License](#license)

## Introduction

This project implements autonomous navigation for a boat using GPS and compass data. The boat is designed to follow a series of waypoints specified by their latitude and longitude coordinates. It calculates the heading angle (alpha) between the current boat position and the target waypoint. A PID controller is used to adjust motor speeds to steer the boat towards the waypoint. The boat stops when it reaches a waypoint, and then proceeds to the next waypoint.

## Requirements

To run this project, you will need the following components:

- Arduino board (tested on Arduino Uno)
- GPS module (tested with a module supporting NMEA sentences at 9600 baud)
- HMC5883L (or similar) compass module
- Motor driver (tested with a dual H-bridge motor driver)
- DC motors (propellers) for boat propulsion
- Power supply for the motors
- Breadboard and jumper wires for prototyping

## Setup

1. Connect the GPS module and HMC5883L compass module to the Arduino board using the appropriate connections (refer to datasheets).
2. Connect the motor driver to the Arduino board and wire it to the DC motors used for boat propulsion.
3. Upload the Arduino code (`main.cpp`) to the Arduino board using the Arduino IDE.
4. Power the system and ensure that the GPS and compass modules are working correctly and providing accurate data.

## Usage

1. Place the boat in an open area with clear GPS signals and calibrate the compass if necessary.
2. The boat will follow the series of waypoints specified in the `latLong` array in the code. You can modify the waypoints to suit your desired route.
3. When the boat reaches a waypoint, it will stop briefly before proceeding to the next waypoint.

## Considerations for Using on Arduino

1. **Hardware Compatibility**: Ensure that your Arduino board, GPS module, compass module, motor driver, and motors are compatible and suitable for your specific setup.

2. **Power Supply**: Make sure to use an appropriate power supply for the motors to avoid overloading the Arduino board's power source.

3. **Debugging**: Debugging Arduino projects can be challenging, especially without a physical display. Consider using Serial communication for debugging purposes. Uncomment and use `Serial.print` statements in the code to observe variable values.

4. **Sensor Accuracy**: GPS and compass modules may have varying levels of accuracy. Calibration and testing in real-world conditions are essential to ensure reliable navigation.

5. **Motor Control**: The PID control logic for motor speed adjustment may need tuning based on your motor characteristics and boat setup.

6. **Safety Precautions**: When testing the boat in water, be cautious to avoid potential accidents. Always have a manual override or emergency stop mechanism in place.

## Google Scholar Profile

You can find more research by the project creator on their [Google Scholar profile](https://scholar.google.co.uk/citations?view_op=view_citation&hl=en&user=JoQWtq0AAAAJ&citation_for_view=JoQWtq0AAAAJ:d1gkVwhDpl0C).

## License

This project is licensed under the [MIT License](LICENSE).

