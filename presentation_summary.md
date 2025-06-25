# Unmanned Fire Detection Robot
## Project Presentation Summary

### Project Objective
Design and implement an unmanned vehicle capable of navigating around building debris and detecting fire sources using the Ackermann steering system and right-hand rule navigation.

---

### Hardware Selection

#### Microcontroller: Arduino Mega 2560
- **Why**: Multiple I/O pins, hardware serial ports, sufficient memory, and processing power
- **Cost**: ~€35

#### Chassis & Motors
- **Configuration**: 4-wheel platform with Ackermann steering
- **Dimensions**: 25cm × 20cm × 10cm
- **Motors**: 2× DC motors (rear) + 1× Servo motor (front steering)
- **Driver**: L298N dual H-bridge
- **Cost**: ~€73

#### Sensors
- **Obstacle Detection**: 3× HC-SR04 ultrasonic sensors (front, 45° left, 45° right)
- **Fire/Gas Detection**: MQ-2 gas sensor (CO and LPG)
- **Position Tracking**: NEO-6M GPS module
- **Orientation**: QMC5883L digital compass
- **Total Sensor Cost**: ~€29

#### Communication
- **Telemetry**: SX1278 LoRa module (433MHz)
- **Range**: Up to 10km line-of-sight
- **Cost**: ~€15

#### Power System
- **Battery**: TATTU 2300mAh 14.8V 75C 4S1P LiPo (270g)
- **Voltage Regulation**: 5V and 6V DC-DC buck converters
- **Operation Time**: ~4.2 hours
- **Cost**: ~€55

#### Total Estimated Cost: ~€227

---

### Navigation Algorithm

1. **Initialization**
   - Record starting position via GPS
   - Calibrate sensors

2. **Right-Hand Rule Implementation**
   - Follow right wall at 20cm distance
   - Adjust speed based on front obstacle distance
   - Decision tree for obstacle avoidance

3. **Fire/Gas Detection**
   - Continuous monitoring of MQ-2 sensor
   - Thresholds: CO > 50ppm, LPG > 2100ppm
   - Emergency telemetry on detection

4. **Telemetry System**
   - 1Hz GPS sampling
   - Format: lat, lon, LPG_ppm, CO_ppm, batt_voltage
   - Regular and emergency transmission modes

---

### Software Architecture

```
Main Loop
  ├── Sensor Reading
  │    ├── Ultrasonic Sensors
  │    ├── Gas Sensor
  │    ├── GPS
  │    ├── Compass
  │    └── Battery Voltage
  │
  ├── Navigation
  │    ├── Wall Following
  │    ├── Obstacle Avoidance
  │    └── Motor Control
  │
  ├── Fire Detection
  │    ├── Gas Level Monitoring
  │    └── Emergency Alerts
  │
  └── Telemetry
       ├── Position Tracking
       ├── Data Formatting
       └── LoRa Transmission
```

---

### Key Features & Advantages

1. **Robust Navigation**
   - Right-hand rule ensures complete exploration
   - Multiple ultrasonic sensors for comprehensive obstacle detection
   - Compass-assisted orientation

2. **Effective Fire Detection**
   - MQ-2 sensor detects both CO and LPG
   - Immediate alert system with GPS coordinates
   - Continuous monitoring during navigation

3. **Long-Range Communication**
   - LoRa technology for reliable long-distance telemetry
   - Standardized data format for easy integration

4. **Power Efficiency**
   - 4.2 hour estimated operation time
   - Battery monitoring and remaining time estimation
   - Optimized power distribution

5. **Modular Design**
   - Easy to upgrade or modify components
   - Well-documented code and connections
   - Uses standard libraries and interfaces

---

### Future Enhancements

1. **Improved Fire Detection**
   - Add thermal camera for visual confirmation
   - Implement smoke detection

2. **Enhanced Navigation**
   - SLAM (Simultaneous Localization and Mapping)
   - Machine learning for adaptive path planning

3. **Extended Operation**
   - Solar charging capability
   - Automatic return to charging station

4. **Advanced Telemetry**
   - Live video streaming
   - Two-way communication for remote control

---

### Conclusion

The unmanned fire detection robot provides a cost-effective solution for exploring hazardous environments and detecting fire sources. With its robust navigation system, comprehensive sensor suite, and reliable communication, it can effectively fulfill its mission of navigating around building debris and identifying potential fire hazards.

The total cost of approximately €227 makes it an economical option compared to commercial alternatives, while the estimated 4.2-hour operation time ensures sufficient duration for typical exploration missions.
