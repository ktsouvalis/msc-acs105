# System Patterns: Fire Detection Robot Architecture

## Overall Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Fire Detection Robot                     │
│                                                             │
│  ┌─────────────────┐    LoRa     ┌─────────────────────────┐ │
│  │  Robot Unit     │◄──────────►│    Base Station         │ │
│  │                 │   433MHz    │                         │ │
│  │ ┌─────────────┐ │             │ ┌─────────────────────┐ │ │
│  │ │   Sensors   │ │             │ │  Emergency Response │ │ │
│  │ │             │ │             │ │    Monitoring       │ │ │
│  │ │ - Gas (MQ-2)│ │             │ │     System          │ │ │
│  │ │ - Ultrasonic│ │             │ └─────────────────────┘ │ │
│  │ │ - GPS       │ │             └─────────────────────────┘ │
│  │ │ - Compass   │ │                                         │
│  │ │ - Battery   │ │                                         │
│  │ └─────────────┘ │                                         │
│  │                 │                                         │
│  │ ┌─────────────┐ │                                         │
│  │ │ Controller  │ │                                         │
│  │ │ Arduino     │ │                                         │
│  │ │ Mega 2560   │ │                                         │
│  │ └─────────────┘ │                                         │
│  │                 │                                         │
│  │ ┌─────────────┐ │                                         │
│  │ │   Motors    │ │                                         │
│  │ │ Ackermann   │ │                                         │
│  │ │ Steering    │ │                                         │
│  │ └─────────────┘ │                                         │
│  └─────────────────┘                                         │
└─────────────────────────────────────────────────────────────┘
```

## Key Design Patterns

### 1. Right-Hand Rule Navigation Pattern
```
State Machine for Navigation:
┌─────────────┐
│    INIT     │──► Sensor calibration, GPS start position
└─────────────┘
       │
       ▼
┌─────────────┐
│   PATROL    │──► Follow right wall, maintain 20cm distance
└─────────────┘
       │
       ▼ (obstacle detected)
┌─────────────┐
│  EVALUATE   │──► Check front, right, left distances
└─────────────┘
       │
       ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ TURN_RIGHT  │     │ TURN_LEFT   │     │ TURN_AROUND │
│ (right open)│     │ (left open) │     │ (both block)│
└─────────────┘     └─────────────┘     └─────────────┘
       │                   │                   │
       └─────────▼─────────────────▼───────────┘
              ┌─────────────┐
              │   PATROL    │
              └─────────────┘
```

### 2. Fire Detection State Pattern
```
┌─────────────┐
│ MONITORING  │──► Continuous MQ-2 sensor reading
└─────────────┘
       │
       ▼ (threshold exceeded)
┌─────────────┐
│ FIRE_DETECT │──► Log GPS coordinates, prepare alert
└─────────────┘
       │
       ▼
┌─────────────┐
│ EMERGENCY   │──► Immediate LoRa transmission
│ TRANSMIT    │    Format: lat,lon,LPG,CO,battery
└─────────────┘
       │
       ▼
┌─────────────┐
│ CONTINUE    │──► Resume normal operation
│ MISSION     │
└─────────────┘
```

### 3. Motor Control Pattern (Ackermann Steering)
```
Drive System Architecture:
┌─────────────────────────────────────────┐
│              Motor Control              │
│                                         │
│  Rear Wheels (Drive)    Front Wheels    │
│  ┌─────────────────┐   ┌─────────────┐  │
│  │   Left Motor    │   │   Servo     │  │
│  │   (ENA, IN1-2)  │   │  Steering   │  │
│  │                 │   │ (Pin 11)    │  │
│  │   Right Motor   │   └─────────────┘  │
│  │   (ENB, IN3-4)  │                    │
│  └─────────────────┘                    │
│                                         │
│  Speed Control: PWM (0-255)             │
│  - Normal: 200 PWM                      │
│  - Slow: 50 PWM                         │
│  - Stop: 0 PWM                          │
│                                         │
│  Steering Angles:                       │
│  - Straight: 90°                        │
│  - Left: 120°                           │
│  - Right: 60°                           │
└─────────────────────────────────────────┘
```

### 4. Sensor Fusion Pattern
```
Sensor Integration Hierarchy:
┌─────────────────────────────────────────┐
│           Sensor Fusion                 │
│                                         │
│  Navigation Sensors    Detection        │
│  ┌─────────────────┐   ┌─────────────┐  │
│  │ HC-SR04 Front   │   │   MQ-2      │  │
│  │ HC-SR04 Right   │   │ Gas Sensor  │  │
│  │ (HC-SR04 Left)  │   └─────────────┘  │
│  └─────────────────┘                    │
│                                         │
│  Position Sensors     Power Monitor     │
│  ┌─────────────────┐   ┌─────────────┐  │
│  │ NEO-6M GPS      │   │ Voltage     │  │
│  │ QMC5883L        │   │ Divider     │  │
│  │ Compass         │   └─────────────┘  │
│  └─────────────────┘                    │
└─────────────────────────────────────────┘

Decision Priority:
1. Safety (obstacle avoidance) - Highest
2. Fire detection (gas thresholds) - High  
3. Navigation (wall following) - Medium
4. Telemetry (position reporting) - Low
```

### 5. Communication Protocol Pattern
```
LoRa Message Structure:
┌─────────────────────────────────────────┐
│        Telemetry Data Format            │
│                                         │
│  Regular Message (1Hz):                 │
│  lat,lon,LPG_ppm,CO_ppm,batt_voltage    │
│                                         │
│  Emergency Message (immediate):         │
│  FIRE!,lat,lon,LPG_ppm,CO_ppm,timestamp │
│                                         │
│  Status Message (on request):           │
│  STATUS,battery_pct,uptime,mode         │
└─────────────────────────────────────────┘

Transmission States:
┌─────────────┐
│   IDLE      │──► Normal operation, no transmission
└─────────────┘
       │
       ▼ (1Hz timer)
┌─────────────┐
│ TRANSMIT    │──► Send telemetry data
│ TELEMETRY   │
└─────────────┘
       │
       ▼ (fire detected)
┌─────────────┐
│ EMERGENCY   │──► Immediate alert transmission
│ ALERT       │
└─────────────┘
```

## Component Relationships

### Core Controller (Arduino Mega 2560)
- **Central Processing**: Main control logic and state machine management
- **Sensor Integration**: Digital/analog interfaces for all sensors
- **Motor Control**: PWM generation for drive motors and servo steering
- **Communication Hub**: Serial interface to LoRa module
- **Memory Management**: Buffer management for sensor data and messages

### Navigation Subsystem
```cpp
// Navigation decision logic
void navigationDecision() {
    int frontDist = sonarFront.ping_cm();
    int rightDist = sonarRight.ping_cm();
    
    if (frontDist > FRONT_SLOW_DIST) {
        followWall(rightDist, NORMAL_SPEED);
    } else if (frontDist <= FRONT_SLOW_DIST && frontDist > FRONT_STOP_DIST) {
        followWall(rightDist, SLOW_SPEED);
    } else if (frontDist <= FRONT_STOP_DIST) {
        if (rightDist < DESIRED_RIGHT_DIST + 2) {
            turnLeft();  // Right blocked, turn left
        } else {
            followWall(rightDist, SLOW_SPEED);  // Follow right opening
        }
    }
}
```

### Sensor Subsystem Architecture
- **Ultrasonic Array**: HC-SR04 sensors for obstacle detection
  - Front sensor: Primary collision avoidance
  - Right sensor: Wall following distance measurement  
  - Left sensor: (Future expansion for enhanced navigation)
- **Gas Detection**: MQ-2 sensor for fire indication
  - Analog input with threshold comparison
  - Continuous sampling with 20-second warmup
- **Position System**: GPS + Compass for location awareness
  - GPS: Absolute positioning for emergency coordinates
  - Compass: Heading reference for navigation corrections

### Power Management System
```
Power Distribution:
14.8V LiPo Battery (2300mAh)
    │
    ├── 6V Buck Converter ──► Servo Motor
    │
    ├── 5V Buck Converter ──► Arduino + Sensors
    │
    └── Variable (via L298N) ──► Drive Motors

Power Monitoring:
- Voltage divider on analog pin
- Real-time battery level calculation
- Low battery warnings in telemetry
```

## Critical Implementation Paths

### 1. Right-Hand Rule Implementation
```cpp
void followWall(int rightDist, int speed) {
    // Maintain desired distance from right wall
    if (rightDist > DESIRED_RIGHT_DIST + 2) {
        steerRight();  // Move closer to wall
    } else if (rightDist < DESIRED_RIGHT_DIST - 2) {
        steerLeft();   // Move away from wall
    } else {
        steerStraight();  // Maintain current path
    }
    moveForward(speed);
}
```

### 2. Fire Detection Algorithm
```cpp
bool detectFire() {
    int gasLevel = analogRead(MQ2_PIN);
    float lpgPPM = calculateLPG(gasLevel);
    float coPPM = calculateCO(gasLevel);
    
    return (lpgPPM > LPG_THRESHOLD || coPPM > CO_THRESHOLD);
}
```

### 3. Emergency Response Protocol
```cpp
void handleFireDetection() {
    float currentLat, currentLon;
    gps.getCoordinates(&currentLat, &currentLon);
    
    String emergencyMsg = "FIRE!" + String(currentLat, 6) + "," 
                        + String(currentLon, 6) + "," 
                        + String(lpgPPM) + "," + String(coPPM);
    
    lora.transmitImmediate(emergencyMsg);
}
```

## Technical Decisions

### Hardware Architecture Choices
- **Arduino Mega 2560**: Selected for sufficient I/O pins and memory
- **Ackermann Steering**: Chosen for precise directional control in confined spaces
- **HC-SR04 Sensors**: Cost-effective ultrasonic sensors with adequate range
- **MQ-2 Gas Sensor**: Multi-gas detection capability (CO and LPG)
- **LoRa SX1278**: Long-range communication with low power consumption

### Software Architecture Decisions
- **State Machine Design**: Clear operational states for predictable behavior
- **Non-blocking Loops**: Prevents sensor reading delays from affecting navigation
- **Modular Functions**: Separate functions for navigation, detection, communication
- **Threshold-based Detection**: Simple but effective fire detection logic
- **Priority-based Processing**: Safety (obstacle avoidance) takes precedence

### Communication Protocol Design
- **Simple CSV Format**: Easy parsing and human-readable telemetry
- **1Hz Regular Updates**: Balance between data freshness and power consumption
- **Immediate Emergency Alerts**: Critical fire detection bypasses normal timing
- **GPS Coordinate Precision**: 6 decimal places for ~1 meter accuracy

### Power Optimization Strategies
- **Speed-based Power Management**: Slower speeds when obstacles detected
- **Sensor Duty Cycling**: Minimize continuous sensor power draw
- **Efficient Motor Control**: PWM-based speed control reduces power waste
- **Communication Optimization**: Minimize transmission frequency while maintaining coverage
