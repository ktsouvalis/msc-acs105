# Unmanned Fire Detection Robot

This repository contains the design and implementation of an unmanned vehicle capable of navigating around building debris and detecting fire sources. The robot uses the Ackermann steering system, follows the right-hand rule for navigation, and is equipped with sensors to detect obstacles and fire/gas.

## Project Overview

The unmanned fire detection robot is designed to:

1. Navigate autonomously around building debris using the right-hand rule
2. Detect fire sources and hazardous gases (CO and LPG)
3. Transmit telemetry data including position and sensor readings
4. Return to its starting point after exploration

## Hardware Components

- **Microcontroller**: Arduino Mega 2560
- **Chassis**: 4-wheel platform with Ackermann steering
- **Sensors**:
  - 3× HC-SR04 ultrasonic sensors (front, 45° left, 45° right)
  - MQ-2 gas sensor for fire/gas detection
  - NEO-6M GPS module
  - QMC5883L digital compass
- **Motors**:
  - 2× DC motors for rear wheels
  - 1× Servo motor for front steering
- **Communication**: SX1278 LoRa module
- **Power**: TATTU 2300mAh 14.8V 75C 4S1P LiPo Battery

## Repository Contents

- [**fire_detection_robot.ino**](fire_detection_robot.ino): Arduino sketch containing the complete code for the robot
- [**fire_detection_robot_v2.ino**](fire_detection_robot_v2.ino): Optimized version using non-blocking timing techniques
- [**fire_detection_robot_circuit.txt**](fire_detection_robot_circuit.txt): Detailed circuit connection instructions for Fritzing
- [**fire_detection_robot_project_overview.md**](fire_detection_robot_project_overview.md): Comprehensive project overview including specifications and cost breakdown
- [**navigation_algorithm.txt**](navigation_algorithm.txt): Visualization and explanation of the navigation algorithm
- [**libraries_and_dependencies.md**](libraries_and_dependencies.md): List of required libraries and installation instructions
- [**code_optimization_notes.md**](code_optimization_notes.md): Explanation of improvements in the optimized code version
- [**presentation_summary.md**](presentation_summary.md): Concise summary for project presentation

## Key Features

- **Obstacle Avoidance**: Uses ultrasonic sensors to detect and navigate around obstacles
- **Wall Following**: Implements the right-hand rule for systematic exploration
- **Fire Detection**: Monitors for CO and LPG gases to detect potential fire sources
- **Telemetry**: Transmits position, gas readings, and battery status via LoRa
- **Battery Management**: Monitors battery voltage and estimates remaining operation time

## Navigation Algorithm

The robot uses the right-hand rule algorithm for navigation:

1. Follow the right wall maintaining a distance of 20cm
2. If front distance > 50cm: Move at normal speed
3. If front distance between 20-50cm: Slow down
4. If front distance < 20cm:
   - If right is open: Turn right
   - If left is open: Turn left
   - If both blocked: Turn around

## Telemetry Format

The robot transmits telemetry data in the following format:
```
lat, lon, LPG_ppm, CO_ppm, batt_voltage
```

## Estimated Specifications

- **Dimensions**: 25cm (L) × 20cm (W) × 10cm (H)
- **Weight**: Approximately 1.2kg
- **Operation Time**: Approximately 4.2 hours
- **Cost**: Approximately 227€

## Setup and Installation

1. Assemble the hardware according to the circuit diagram
2. Install the required libraries as listed in [libraries_and_dependencies.md](libraries_and_dependencies.md)
3. Upload the [fire_detection_robot.ino](fire_detection_robot.ino) sketch to the Arduino Mega
4. Calibrate the sensors as needed
5. Power on and place the robot at the starting position

## Future Improvements

- Camera integration for visual fire detection
- Thermal sensor for more accurate fire detection
- Path optimization for more efficient return journey
- Autonomous charging capability for extended missions

## License

This project is open source and available under the MIT License.

## Acknowledgments

- This project was developed as part of the MSC-ACS105 course
- Special thanks to all contributors and the open-source community for the libraries used
