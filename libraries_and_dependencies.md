# Libraries and Dependencies for the Fire Detection Robot

This document provides information about all the libraries required for the unmanned fire detection robot project, along with installation instructions.

## Required Libraries

The project uses the following libraries:

1. **NewPing** - For ultrasonic sensors
2. **Servo** - For controlling the steering servo
3. **TinyGPS++** - For GPS module
4. **LoRa** - For SX1278 LoRa module
5. **Wire** - For I2C communication (built-in)
6. **QMC5883LCompass** - For digital compass

## Installation Instructions

### Using Arduino IDE Library Manager

For most libraries, you can use the Arduino IDE Library Manager:

1. Open Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries...**
3. Search for each library by name
4. Click "Install" for each required library

### Manual Installation

If a library is not available in the Library Manager, you can install it manually:

1. Download the library as a ZIP file from its GitHub repository
2. In Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library...**
3. Select the downloaded ZIP file

## Library Details and Sources

### 1. NewPing Library
- **Purpose**: Interface with HC-SR04 ultrasonic sensors
- **Version**: 1.9.7 or later
- **Author**: Tim Eckel
- **GitHub**: [https://github.com/teckel12/NewPing](https://github.com/teckel12/NewPing)
- **Installation**: Available in Library Manager

### 2. Servo Library
- **Purpose**: Control servo motors
- **Version**: 1.2.2 or later
- **Author**: Arduino
- **Installation**: Built-in with Arduino IDE

### 3. TinyGPS++ Library
- **Purpose**: Parse NMEA data from GPS modules
- **Version**: 1.0.3 or later
- **Author**: Mikal Hart
- **GitHub**: [https://github.com/mikalhart/TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)
- **Installation**: Available in Library Manager

### 4. LoRa Library
- **Purpose**: Interface with SX1278 LoRa modules
- **Version**: 0.8.0 or later
- **Author**: Sandeep Mistry
- **GitHub**: [https://github.com/sandeepmistry/arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa)
- **Installation**: Available in Library Manager

### 5. Wire Library
- **Purpose**: I2C communication
- **Version**: Built-in
- **Author**: Arduino
- **Installation**: Built-in with Arduino IDE

### 6. QMC5883LCompass Library
- **Purpose**: Interface with QMC5883L digital compass
- **Version**: 1.0.0 or later
- **Author**: MPrograms
- **GitHub**: [https://github.com/mprograms/QMC5883LCompass](https://github.com/mprograms/QMC5883LCompass)
- **Installation**: Download ZIP from GitHub and use "Add .ZIP Library..." option

## PlatformIO Installation

If you're using PlatformIO instead of Arduino IDE, add these dependencies to your `platformio.ini` file:

```ini
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
lib_deps = 
    arduino-libraries/Servo@^1.2.2
    teckel12/NewPing@^1.9.7
    mikalhart/TinyGPSPlus@^1.0.3
    sandeepmistry/LoRa@^0.8.0
    mprograms/QMC5883LCompass@^1.0.0
```

## Troubleshooting Common Issues

### Missing Libraries
If you see errors like `cannot open source file "LibraryName.h"`, make sure you've installed all the required libraries.

### Incompatible Versions
If you encounter compatibility issues, try installing specific versions of libraries as listed above.

### Arduino Mega 2560 Board Support
Ensure you have installed the Arduino AVR Boards package:
1. Go to **Tools > Board > Boards Manager...**
2. Search for "Arduino AVR Boards"
3. Install or update to the latest version

### Library Conflicts
If you experience conflicts between libraries:
1. Check for duplicate libraries in your Arduino libraries folder
2. Remove older versions
3. Restart the Arduino IDE

## Additional Resources

- [Arduino Library Installation Guide](https://www.arduino.cc/en/Guide/Libraries)
- [PlatformIO Library Management](https://docs.platformio.org/en/latest/librarymanager/index.html)
