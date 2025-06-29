# Technical Context: Fire Detection Robot

## Development Environment

### Platform
- **PlatformIO**: Primary development environment
- **Framework**: Arduino framework for ATmega2560
- **IDE Integration**: VSCode with PlatformIO extension
- **Version Control**: Git with GitHub repository
- **Target Board**: Arduino Mega 2560 (megaatmega2560)

### Current Configuration
```ini
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
lib_deps = 
    arduino-libraries/Servo@^1.2.2
    teckel12/NewPing@^1.9.7
```

### Target Hardware
- **Primary MCU**: Arduino Mega 2560 (ATmega2560)
- **Clock Speed**: 16MHz
- **Memory**: 256KB Flash, 8KB SRAM, 4KB EEPROM
- **I/O Pins**: 54 digital pins, 16 analog inputs
- **PWM Pins**: 15 pins with PWM capability
- **Serial Ports**: 4 hardware UART ports

## Technology Stack

### Core Technologies
- **C++**: Primary programming language
- **Arduino Framework**: Hardware abstraction layer
- **PlatformIO**: Build system and library management
- **AVR-GCC**: Compiler toolchain for ATmega2560

### Communication Protocols
- **LoRa**: Long-range wireless communication
  - Frequency: 433MHz
  - Range: Up to 10km line-of-sight
  - Protocol: Custom CSV message format
  - Module: SX1278-based transceiver
- **I2C**: Digital compass and potential sensor expansion
- **SPI**: Future LoRa module integration
- **UART**: GPS module communication
- **Analog**: Gas sensor and battery monitoring

### Libraries and Dependencies

#### Current Dependencies
- **Servo Library (v1.2.2)**: Arduino-libraries/Servo
  - Controls steering servo motor
  - Provides precise angle control (60°, 90°, 120°)
- **NewPing Library (v1.9.7)**: teckel12/NewPing
  - Ultrasonic sensor interface
  - Handles HC-SR04 timing and distance calculation
  - Multiple sensor support

#### Planned Dependencies (Not Yet Integrated)
- **TinyGPS++**: GPS NMEA sentence parsing
- **SoftwareSerial**: Additional serial communication
- **LoRa Library**: SX1278 module communication
- **QMC5883L Library**: Digital compass interface
- **Math Libraries**: Navigation calculations

### Hardware Interfaces

#### Pin Assignments (Current Implementation)
```cpp
// Ultrasonic Sensors
#define TRIG_FRONT 2
#define ECHO_FRONT 3
#define TRIG_RIGHT 4
#define ECHO_RIGHT 5
// Future: TRIG_LEFT 6, ECHO_LEFT 7

// Motor Control (L298N Driver)
#define ENA 9   // Left motor PWM
#define IN1 7   // Left motor direction
#define IN2 8   // Left motor direction
#define ENB 6   // Right motor PWM
#define IN3 12  // Right motor direction
#define IN4 13  // Right motor direction

// Servo Steering
#define SERVO_PIN 11

// Future Expansion
// #define MQ2_PIN A0      // Gas sensor
// #define GPS_RX 18       // GPS module
// #define GPS_TX 19       // GPS module
// #define LORA_NSS 53     // LoRa chip select
// #define BATTERY_PIN A1   // Battery voltage
```

#### Power Distribution
- **5V Rail**: Arduino, sensors, logic circuits
- **6V Rail**: Servo motor (via buck converter)
- **Variable (6-14.8V)**: Drive motors (via L298N)
- **14.8V**: Direct battery connection for power monitoring

## Development Setup

### Required Tools
- **PlatformIO Core**: Command-line build system
- **VSCode**: Integrated development environment
- **PlatformIO IDE Extension**: VSCode integration
- **Git**: Version control system
- **Arduino IDE**: Backup development environment

### Build Configuration
```ini
# platformio.ini - Current Configuration
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
monitor_speed = 9600
upload_speed = 57600

# Future expansion for ESP32 version
# [env:esp32dev]
# platform = espressif32
# board = esp32dev
# framework = arduino
# monitor_speed = 115200
```

### Development Workflow
1. **Code Development**: VSCode with PlatformIO extension
2. **Build**: `pio run` for compilation
3. **Upload**: `pio run --target upload`
4. **Monitor**: `pio device monitor` for serial debugging
5. **Test**: Hardware-in-the-loop testing with breadboard setup

## Technical Constraints

### Hardware Limitations
- **Processing Power**: 16MHz ATmega2560 limits complex calculations
- **Memory Constraints**: 8KB SRAM limits buffer sizes and variables
- **Power Consumption**: ~8.1W total system consumption
- **Communication Range**: LoRa range affected by terrain and obstacles
- **Sensor Accuracy**: HC-SR04 limited to ~2cm accuracy
- **Weight Limit**: 1.2kg total system weight constraint

### Environmental Factors
- **Temperature Range**: -40°C to +85°C for electronics
- **Humidity**: Electronics require protection in outdoor environments
- **Electromagnetic Interference**: 433MHz susceptible to interference
- **GPS Signal**: Requires clear sky view for accuracy
- **Obstacle Material**: Ultrasonic sensors affected by sound-absorbing materials

### Software Constraints
- **Real-time Requirements**: 100ms main loop timing
- **Memory Management**: Careful string and buffer management
- **Blocking Operations**: Avoid delays that interrupt navigation
- **Power Optimization**: Minimize unnecessary sensor readings
- **Error Handling**: Robust operation despite sensor failures

## Current Implementation Status

### Implemented Features
- **Basic Navigation**: Right-hand rule algorithm
- **Obstacle Detection**: Front and right ultrasonic sensors
- **Motor Control**: PWM-based speed control with Ackermann steering
- **Wall Following**: Proportional distance control
- **Serial Debugging**: 9600 baud monitoring interface

### Navigation Algorithm (Current)
```cpp
void loop() {
  int frontDist = sonarFront.ping_cm();
  int rightDist = sonarRight.ping_cm();

  if (frontDist > FRONT_SLOW_DIST) {
    followWall(rightDist, NORMAL_SPEED);
  } else if (frontDist <= FRONT_SLOW_DIST && frontDist > FRONT_STOP_DIST) {
    followWall(rightDist, SLOW_SPEED);
  } else if (frontDist <= FRONT_STOP_DIST) {
    if (rightDist < DESIRED_RIGHT_DIST + 2) {
      turnLeft();
    } else {
      followWall(rightDist, SLOW_SPEED);
    }
  }
  delay(100);
}
```

### Missing Integrations
- **Fire Detection**: MQ-2 gas sensor not yet integrated
- **GPS Navigation**: NEO-6M GPS module not connected
- **LoRa Communication**: SX1278 module not implemented
- **Digital Compass**: QMC5883L compass not integrated
- **Battery Monitoring**: Voltage divider circuit not implemented

## Integration Patterns

### Sensor Integration Strategy
- **Polling-based**: Current ultrasonic sensor reading
- **Interrupt-driven**: Future implementation for time-critical sensors
- **Calibration Routines**: Sensor-specific calibration on startup
- **Error Detection**: Timeout and range validation for all sensors

### Communication Integration Plan
```cpp
// Future LoRa integration structure
struct TelemetryData {
    float latitude;
    float longitude;
    float lpgPPM;
    float coPPM;
    float batteryVoltage;
    unsigned long timestamp;
};

void transmitTelemetry(TelemetryData data) {
    String message = String(data.latitude, 6) + "," +
                    String(data.longitude, 6) + "," +
                    String(data.lpgPPM, 2) + "," +
                    String(data.coPPM, 2) + "," +
                    String(data.batteryVoltage, 2);
    lora.transmit(message);
}
```

### Navigation Enhancement Requirements
- **GPS Integration**: Calculate bearing to waypoints
- **Compass Integration**: Maintain heading awareness
- **Path Memory**: Record visited locations to avoid loops
- **Return Navigation**: GPS-guided return to start point

## Performance Considerations

### Current Performance Metrics
- **Loop Frequency**: ~10Hz (100ms delay)
- **Sensor Update Rate**: 10Hz for ultrasonic sensors
- **Motor Response Time**: Immediate PWM changes
- **Turn Execution Time**: 600ms for left turn maneuver

### Optimization Targets
- **Battery Life**: Target 4+ hours of continuous operation
- **Response Time**: <200ms from obstacle detection to evasive action
- **Navigation Precision**: ±2cm wall following accuracy
- **Communication Latency**: <1s for emergency fire alerts

### Memory Optimization
- **String Handling**: Minimize dynamic string allocation
- **Buffer Management**: Fixed-size buffers for sensor data
- **Function Optimization**: Inline critical navigation functions
- **Library Selection**: Choose memory-efficient libraries

## Future Migration Path

### ESP32 Upgrade Considerations
- **Enhanced Processing**: 240MHz dual-core vs 16MHz single-core
- **WiFi Capability**: Additional communication channel
- **More Memory**: 520KB SRAM vs 8KB current
- **Better ADC**: 12-bit vs 10-bit analog resolution
- **Multiple UARTs**: Better sensor integration capability

### Scalability Considerations
- **Multi-Robot Coordination**: Mesh networking capability
- **Advanced Sensors**: LIDAR, thermal imaging integration
- **Machine Learning**: Pattern recognition for fire detection
- **Cloud Integration**: IoT platform connectivity
- **Real-time Video**: Camera streaming capability

## Debugging and Monitoring

### Current Debug Capabilities
- **Serial Monitor**: 9600 baud debug output
- **LED Indicators**: Visual status indicators (planned)
- **Distance Reporting**: Real-time sensor values
- **Motor Status**: Speed and direction feedback

### Enhanced Monitoring (Planned)
- **Remote Diagnostics**: LoRa-based status reporting
- **Performance Metrics**: Battery usage, sensor accuracy
- **Error Logging**: Persistent error storage in EEPROM
- **Mission Recording**: GPS track and sensor data logging
