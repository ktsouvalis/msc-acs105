# Unmanned Fire Detection Robot - Project Overview

## 1. Introduction

This document provides a comprehensive overview of the unmanned fire detection robot designed to navigate around building debris and detect fire sources. The robot uses the Ackermann steering system, ultrasonic sensors for obstacle detection, and follows the right-hand rule for navigation.

## 2. Microcontroller Selection

**Arduino Mega 2560** is selected as the microcontroller for this project for the following reasons:

- **Sufficient I/O pins**: The project requires numerous digital and analog pins for sensors, motors, and communication modules
- **Multiple hardware serial ports**: Needed for GPS and debugging
- **Large memory**: 256KB flash memory and 8KB SRAM provide ample space for the navigation algorithms and sensor processing
- **Processing power**: 16MHz ATmega2560 processor is sufficient for the required tasks
- **Community support**: Extensive libraries and documentation available
- **Cost-effective**: Approximately €35, offering good value for the capabilities

## 3. Chassis and Hardware Specifications

### Chassis
- **Type**: 4-wheel platform with Ackermann steering
- **Dimensions**: 25cm (L) × 20cm (W) × 10cm (H)
- **Wheel diameter**: 65mm
- **Total weight**: Approximately 1.2kg (including all components)

### Motors and Steering
- **Rear wheels**: 2× DC motors (125 RPM at 100% duty cycle)
- **Front wheels**: 1× Servo motor for Ackermann steering
- **Motor driver**: L298N dual H-bridge motor driver

### Sensors
- **Obstacle detection**: 3× HC-SR04 ultrasonic sensors
  - Front-facing
  - 45° left
  - 45° right
- **Fire/gas detection**: MQ-2 gas sensor (detects CO and LPG)
- **Position tracking**: NEO-6M GPS module
- **Orientation**: QMC5883L digital compass
- **Power monitoring**: Voltage divider for battery level

### Communication
- **Telemetry**: SX1278 LoRa module (433MHz)
  - Range: Up to 10km line-of-sight
  - Data format: `lat, lon, LPG_ppm, CO_ppm, batt_voltage`

## 4. Power System

### Battery
- **Type**: TATTU 2300mAh 14.8V 75C 4S1P LiPo Battery
- **Weight**: 270g
- **Capacity**: 34.04Wh (2300mAh × 14.8V)

### Power Distribution
- **Main controller**: 5V DC-DC buck converter for Arduino and sensors
- **Servo power**: 6V DC-DC buck converter for servo motor
- **Motor power**: Direct from battery via L298N driver

### Power Consumption Estimates
- Arduino Mega: ~100mA at 5V = 0.5W
- Motors (2×): ~500mA each at 6V = 6W
- Servo: ~100mA at 6V = 0.6W
- Sensors and modules: ~200mA at 5V = 1W
- **Total consumption**: ~8.1W

### Estimated Operation Time
- 34.04Wh ÷ 8.1W ≈ 4.2 hours

## 5. Cost Breakdown

| Component | Quantity | Est. Cost (€) |
|-----------|----------|--------------|
| Arduino Mega 2560 | 1 | 35 |
| Chassis with motors | 1 | 60 |
| L298N Motor Driver | 1 | 5 |
| HC-SR04 Ultrasonic Sensors | 3 | 9 |
| MQ-2 Gas Sensor | 1 | 4 |
| NEO-6M GPS Module | 1 | 12 |
| SX1278 LoRa Module | 1 | 15 |
| QMC5883L Digital Compass | 1 | 4 |
| Servo Motor | 1 | 8 |
| DC-DC Buck Converters | 2 | 10 |
| TATTU 2300mAh 14.8V LiPo | 1 | 45 |
| Wires, connectors, misc | - | 20 |
| **Total** | | **~227€** |

## 6. Software Architecture

The software is structured into several functional modules:

### Navigation System
- **Wall following**: Implements the right-hand rule algorithm
- **Obstacle detection**: Processes ultrasonic sensor data
- **Motor control**: Manages DC motors and servo for Ackermann steering
- **Orientation**: Uses compass data to maintain heading awareness

### Sensor Management
- **Ultrasonic sensors**: Distance measurement and obstacle detection
- **Gas sensor**: Fire and gas detection with thresholds
- **GPS**: Position tracking with 1Hz sampling
- **Compass**: Heading determination
- **Battery**: Voltage monitoring and remaining time estimation

### Telemetry System
- **Regular updates**: 1Hz transmission of position, gas readings, and battery status
- **Emergency alerts**: Immediate transmission when fire/gas is detected
- **Data formatting**: Standardized format for base station processing

## 7. Navigation Algorithm

The robot uses the right-hand rule algorithm for navigation:

1. **Initialization**:
   - Record starting position via GPS
   - Warm up sensors
   - Calibrate compass

2. **Main Navigation Loop**:
   - Follow the right wall maintaining a distance of 20cm
   - If front distance > 50cm: Move at normal speed (100% duty cycle)
   - If front distance between 20-50cm: Slow down (10% duty cycle)
   - If front distance < 20cm:
     - If right is open: Turn right
     - If left is open: Turn left
     - If both blocked: Turn around

3. **Fire/Gas Detection**:
   - Continuously monitor MQ-2 sensor
   - If CO > 50ppm or LPG > 2100ppm:
     - Record GPS position
     - Send emergency telemetry
     - Continue mission

4. **Return Journey**:
   - After completing exploration or on command
   - Use recorded GPS coordinates to navigate back to starting point

## 8. Implementation Notes

### Sensor Calibration
- **MQ-2 Sensor**: Requires 20-second warmup period
- **Compass**: Needs calibration to account for local magnetic variations
- **Ultrasonic Sensors**: Positioned to provide optimal coverage

### Safety Features
- **Battery monitoring**: Low voltage warnings
- **Obstacle avoidance**: Multiple sensors for redundancy
- **Emergency stop**: Can be implemented via LoRa command (not in current code)

### Potential Improvements
- **Camera integration**: For visual fire detection
- **Thermal sensor**: For more accurate fire detection
- **Path optimization**: More sophisticated return path algorithm
- **Autonomous charging**: Docking capability for extended missions

## 9. Conclusion

This unmanned fire detection robot provides a cost-effective solution for exploring hazardous environments and detecting fire sources. With an estimated operation time of 4.2 hours and a comprehensive sensor suite, it can effectively navigate around building debris using the right-hand rule algorithm while monitoring for fire and gas.

The modular design allows for future enhancements and adaptations to different scenarios, making it a versatile platform for disaster response applications.
