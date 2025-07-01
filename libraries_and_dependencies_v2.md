# Libraries and Dependencies - Enhanced Version 2

This document lists all required libraries and dependencies for the enhanced fire detection robot with INA260 current monitoring and GPS PPS integration.

## Required Arduino Libraries

### Core Libraries (Built-in with Arduino IDE)
- **Wire.h** - I2C communication (built-in)
- **SPI.h** - SPI communication (built-in)
- **SD.h** - SD card functionality (built-in)
- **Servo.h** - Servo motor control (built-in)

### External Libraries (Must be installed)

#### 1. **NewPing Library**
- **Purpose:** HC-SR04 ultrasonic sensor control
- **Version:** 1.9.4 or later
- **Installation:** Arduino Library Manager → Search "NewPing"
- **Author:** Tim Eckel
- **GitHub:** https://github.com/teckel12/NewPing

#### 2. **TinyGPS++ Library**
- **Purpose:** GPS module parsing and enhanced functionality
- **Version:** 1.0.3 or later
- **Installation:** Arduino Library Manager → Search "TinyGPS++"
- **Author:** Mikal Hart
- **GitHub:** https://github.com/mikalhart/TinyGPSPlus

#### 3. **LoRa Library**
- **Purpose:** SX1278 LoRa module communication
- **Version:** 0.8.0 or later
- **Installation:** Arduino Library Manager → Search "LoRa" by Sandeep Mistry
- **Author:** Sandeep Mistry
- **GitHub:** https://github.com/sandeepmistry/arduino-LoRa

#### 4. **INA260_WE Library** (NEW - CRITICAL)
- **Purpose:** INA260 current/voltage/power sensor
- **Version:** 1.2.8 or later
- **Installation:** Arduino Library Manager → Search "INA260_WE"
- **Author:** Wolfgang Ewald
- **GitHub:** https://github.com/wollewald/INA260_WE
- **Features:**
  - High precision current measurement (±0.15%)
  - Voltage and power calculation
  - I2C communication
  - Configurable averaging and conversion times

#### 5. **QMC5883LCompass Library** (Optional - if compass still used)
- **Purpose:** QMC5883L digital compass
- **Version:** 1.1.1 or later
- **Installation:** Arduino Library Manager → Search "QMC5883LCompass"
- **Author:** MPrograms
- **Note:** May be removed if compass functionality is not needed

## Hardware Requirements

### Microcontroller
- **Arduino Mega 2560** (required for sufficient pins and memory)
- **Flash Memory:** 256KB (enhanced code requires more space)
- **SRAM:** 8KB (multiple libraries and variables)
- **EEPROM:** 4KB (for configuration storage)

### Sensors and Modules

#### Power Monitoring (NEW)
- **INA260 Current/Voltage/Power Sensor**
  - I2C Address: 0x40 (default)
  - Supply Voltage: 2.7V to 5.5V
  - Current Range: ±15A
  - Voltage Range: 0V to 36V
  - Resolution: 16-bit ADC
  - Accuracy: ±0.15% (typical)

#### GPS Module (Enhanced)
- **NEO-6M GPS Module with PPS pin**
  - UART Communication: 9600 baud
  - **PPS Output:** 1Hz square wave for timing
  - Update Rate: 1Hz (configurable up to 10Hz)
  - Accuracy: 2.5m CEP (with PPS: sub-meter possible)
  - Cold Start: 27s, Hot Start: 1s

#### Existing Sensors
- **3× HC-SR04 Ultrasonic Sensors**
- **MQ-2 Gas Sensor**
- **SX1278 LoRa Module**
- **SD Card Module**
- **Servo Motor**
- **L298N Motor Driver**

### Power System
- **TATTU 2300mAh 14.8V LiPo Battery (4S)**
  - Nominal Voltage: 14.8V (3.7V per cell)
  - Capacity: 2300mAh
  - Discharge Rate: 25C continuous
  - Configuration: 4S1P (4 cells in series)

## Installation Instructions

### 1. Arduino IDE Setup
```bash
# Install Arduino IDE 1.8.19 or later
# Or Arduino IDE 2.0+ for better performance
```

### 2. Library Installation via Arduino IDE
```
Tools → Manage Libraries → Library Manager

Search and install:
1. "NewPing" by Tim Eckel
2. "TinyGPS++" by Mikal Hart  
3. "LoRa" by Sandeep Mistry
4. "INA260_WE" by Wolfgang Ewald
5. "QMC5883LCompass" by MPrograms (optional)
```

### 3. Manual Installation (if needed)
```bash
# Download libraries from GitHub
# Extract to Arduino/libraries/ folder
# Restart Arduino IDE
```

### 4. Board Configuration
```
Board: Arduino Mega or Mega 2560
Processor: ATmega2560 (Mega 2560)
Port: [Select appropriate COM port]
Programmer: AVRISP mkII
```

## Library Dependencies and Compatibility

### INA260_WE Library Dependencies
- **Wire.h** (I2C communication)
- **Arduino.h** (core Arduino functions)
- Compatible with Arduino IDE 1.6.0+
- Supports ESP32, ESP8266, Arduino Uno/Mega

### Memory Usage Estimation
```
Enhanced Code (V3):
- Flash Memory: ~45KB (18% of 256KB)
- SRAM: ~3.2KB (40% of 8KB)
- Libraries: ~15KB additional flash
```

### I2C Address Map
```
0x40: INA260 Current/Voltage/Power Sensor
0x0D: QMC5883L Digital Compass (if used)
0x48-0x4F: Available for additional I2C devices
```

## Configuration Constants

### INA260 Configuration
```cpp
// I2C Address
#define INA260_ADDRESS 0x40

// Measurement Configuration
ina260.setMeasureMode(CONTINUOUS);
ina260.setConversionTime(CONV_TIME_1100);  // 1.1ms conversion
ina260.setAverage(AVERAGE_16);             // 16-sample averaging
```

### GPS PPS Configuration
```cpp
// PPS Pin Assignment
#define GPS_PPS_PIN 2  // Interrupt pin INT0

// GPS Accuracy Thresholds
#define GPS_MIN_ACCURACY 5.0    // meters
#define GPS_MIN_SATELLITES 4    // minimum satellites
```

### Battery Management Constants
```cpp
// Battery Specifications
#define BATTERY_CAPACITY_MAH 2300.0
#define CELL_COUNT 4.0
#define CELL_VOLTAGE_MAX 4.2
#define CELL_VOLTAGE_MIN 3.3
```

## Troubleshooting

### Common Issues

#### 1. INA260 Not Detected
```
Error: "INA260 initialization failed!"
Solutions:
- Check I2C wiring (SDA: Pin 20, SCL: Pin 21)
- Verify 5V power supply to INA260
- Check I2C address (default 0x40)
- Add 4.7kΩ pull-up resistors on SDA/SCL
```

#### 2. GPS PPS Not Working
```
Issue: No PPS interrupt received
Solutions:
- Verify GPS module has PPS pin
- Check connection to Pin 2 (INT0)
- Ensure GPS has valid fix (4+ satellites)
- Test with oscilloscope (1Hz square wave)
```

#### 3. Memory Issues
```
Error: "Low memory available"
Solutions:
- Use Arduino Mega 2560 (not Uno)
- Optimize string usage
- Remove unused libraries
- Use PROGMEM for constants
```

#### 4. Library Conflicts
```
Error: Multiple library versions
Solutions:
- Remove old library versions
- Clear Arduino cache
- Reinstall libraries in correct order
- Check library compatibility
```

## Performance Optimizations

### 1. I2C Bus Speed
```cpp
// Increase I2C clock speed for faster sensor readings
Wire.setClock(400000);  // 400kHz (fast mode)
```

### 2. GPS Update Rate
```cpp
// Configure GPS for higher update rate if needed
// Send NMEA command: $PMTK220,100*2F (10Hz)
```

### 3. Battery Monitoring Frequency
```cpp
// Optimize battery reading frequency
#define BATTERY_INTERVAL 100  // 100ms for accurate coulomb counting
```

## Version Compatibility

### Arduino IDE Versions
- **Minimum:** Arduino IDE 1.8.0
- **Recommended:** Arduino IDE 1.8.19 or Arduino IDE 2.0+
- **Tested:** Arduino IDE 1.8.19, 2.0.3

### Library Versions
- **NewPing:** 1.9.4+ (tested with 1.9.7)
- **TinyGPS++:** 1.0.3+ (tested with 1.0.3)
- **LoRa:** 0.8.0+ (tested with 0.8.0)
- **INA260_WE:** 1.2.8+ (tested with 1.2.8)

### Hardware Compatibility
- **Arduino Mega 2560:** Full compatibility
- **Arduino Uno:** Not recommended (insufficient pins/memory)
- **ESP32:** Possible with pin mapping changes
- **ESP8266:** Not recommended (insufficient pins)

This enhanced library configuration provides professional-grade power monitoring and GPS accuracy for critical autonomous missions.
