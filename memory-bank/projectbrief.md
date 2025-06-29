# Project Brief: UVS Fire Detection Robot

## Project Overview
Development of an autonomous fire detection robot system for the University of Western Attica (UVS). This project combines robotics, IoT, and fire safety technologies to create a mobile platform capable of detecting fires and transmitting critical data.

## Core Requirements
1. **Fire Detection**: Multi-sensor approach using flame sensors, gas sensors (MQ-2 for CO and LPG), and temperature monitoring
2. **Autonomous Navigation**: GPS-based navigation with compass heading and right-hand rule algorithm for obstacle avoidance
3. **Wireless Communication**: LoRa-based telemetry system for remote monitoring and data transmission
4. **Mobile Platform**: Tracked/wheeled robot chassis with Ackermann steering for terrain traversal
5. **Real-time Monitoring**: Continuous sensor data collection and transmission

## Primary Goals
- Create a functional prototype for fire detection in hazardous environments (building debris)
- Implement reliable wireless communication for emergency response
- Develop autonomous navigation capabilities using right-hand rule algorithm
- Integrate multiple sensor types for accurate fire detection
- Provide real-time telemetry data to monitoring stations

## Technical Scope
- Arduino Mega 2560-based embedded system (current implementation)
- PlatformIO development environment
- LoRa wireless communication protocol (433MHz, up to 10km range)
- GPS navigation system with digital compass
- Multi-sensor fire detection array (MQ-2 gas sensor, ultrasonic sensors)
- Motor control for autonomous movement with Ackermann steering

## Success Criteria
- Successful fire detection with minimal false positives (CO > 50ppm, LPG > 2100ppm thresholds)
- Reliable LoRa communication range and data transmission
- Autonomous navigation around building debris using right-hand rule
- Integration of all subsystems into cohesive platform
- Demonstration-ready prototype for academic presentation
- 4+ hour operation time on single battery charge

## Project Context
This is an academic project for MSC-ACS105 at the University of Western Attica, focusing on practical application of embedded systems, IoT technologies, and robotics in fire safety applications. The robot is designed to navigate dangerous environments where human access is limited or unsafe.

## Key Constraints
- Budget constraint: ~€227 total cost
- Size constraint: 25cm × 20cm × 10cm maximum dimensions
- Weight constraint: ~1.2kg total weight
- Power constraint: 4.2 hours operation time with 2300mAh LiPo battery
- Communication constraint: LoRa-only for remote telemetry
