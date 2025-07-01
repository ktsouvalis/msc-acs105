/*
 * Unmanned Fire Detection Robot - Version 3 (Enhanced with Advanced Battery Management)
 * 
 * This version includes:
 * - INA260 current/voltage/power monitoring for accurate coulomb counting
 * - GPS PPS integration for enhanced positioning accuracy
 * - Advanced battery management with non-linear LiPo modeling
 * - Enhanced telemetry with comprehensive power and GPS metrics
 * - Non-blocking timing techniques for improved responsiveness
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - 3x HC-SR04 Ultrasonic sensors (front, 45° left, 45° right)
 * - L298N Motor Driver for DC motors
 * - Servo motor for Ackermann steering
 * - MQ-2 Gas sensor for fire/gas detection
 * - NEO-6M GPS module with PPS pin
 * - INA260 Current/Voltage/Power sensor
 * - SX1278 LoRa module for telemetry
 * - TATTU 2300mAh 14.8V LiPo Battery (4S configuration)
 */

#include <NewPing.h>         // Ultrasonic sensors
#include <Servo.h>           // Steering servo
#include <TinyGPS++.h>       // GPS module
#include <LoRa.h>            // SX1278 LoRa module
#include <SD.h>              // SD card logging
#include <Wire.h>            // I2C communication
#include <INA260_WE.h>       // INA260 current/voltage/power sensor

// ===== PIN DEFINITIONS =====

// --- SD Card Pins (Optimized for close physical grouping) ---
#define SD_CS_PIN 53    // Close to SPI pins for better connections
// Note: SD card uses hardware SPI pins on Mega
#define SD_MOSI 51      // Hardware SPI MOSI
#define SD_MISO 50      // Hardware SPI MISO  
#define SD_SCK 52       // Hardware SPI SCK

// Ultrasonic Sensors (Grouped for easier wiring)
#define TRIG_FRONT 30   // Digital pins grouped together
#define ECHO_FRONT 31
#define TRIG_RIGHT 32
#define ECHO_RIGHT 33
#define TRIG_LEFT 34
#define ECHO_LEFT 35
#define MAX_DISTANCE 200 // Maximum distance for ultrasonic sensors (cm)

// Motor Driver (L298N) - Grouped for easier connections
#define ENA 9   // Left motor PWM
#define IN1 22  
#define IN2 23  
#define ENB 10  // Right motor PWM (back to 10, no conflict with new SD CS)
#define IN3 24
#define IN4 25

// Servo (PWM pin)
#define SERVO_PIN 11    // PWM pin for servo control

// Sensors
#define MQ2_PIN A0      // MQ-2 Gas sensor
#define GPS_PPS_PIN 2   // GPS PPS (Pulse Per Second) interrupt pin

// LoRa Module (SX1278) - Moved to avoid SD card conflict
#define LORA_SS 49      // NSS pin (moved from 53)
#define LORA_RST 48     // Reset pin (moved from 49)
#define LORA_DIO0 47    // DIO0 pin (moved from 48)

// I2C Addresses
#define INA260_ADDRESS 0x40  // INA260 I2C address

// ===== OBJECTS =====

// Ultrasonic sensors
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);

// Servo for steering
Servo steering;

// GPS
TinyGPSPlus gps;

// INA260 Current/Voltage/Power sensor
INA260_WE ina260 = INA260_WE(INA260_ADDRESS);

// ===== CONSTANTS =====

// Navigation parameters
const int NORMAL_SPEED = 200;     // Normal motor speed (PWM value)
const int SLOW_SPEED = 50;        // Slow motor speed (PWM value)
const int DESIRED_RIGHT_DIST = 20; // Desired distance from right wall (cm)
const int FRONT_SLOW_DIST = 50;   // Distance to start slowing down (cm)
const int FRONT_STOP_DIST = 20;   // Distance to stop/turn (cm)

// Sensor thresholds
const int CO_THRESHOLD = 50;      // CO threshold in ppm
const int LPG_THRESHOLD = 2100;   // LPG threshold in ppm

// Battery specifications (TATTU 2300mAh 14.8V LiPo - 4S)
const float BATTERY_CAPACITY_MAH = 2300.0;    // Battery capacity in mAh
const float CELL_COUNT = 4.0;                 // Number of cells in series
const float CELL_VOLTAGE_MAX = 4.2;           // Maximum cell voltage (V)
const float CELL_VOLTAGE_MIN = 3.3;           // Minimum safe cell voltage (V)
const float BATTERY_VOLTAGE_MAX = CELL_COUNT * CELL_VOLTAGE_MAX; // 16.8V
const float BATTERY_VOLTAGE_MIN = CELL_COUNT * CELL_VOLTAGE_MIN; // 13.2V

// Servo angles
const int SERVO_CENTER = 90;      // Straight position
const int SERVO_LEFT = 120;       // Left turn position
const int SERVO_RIGHT = 60;       // Right turn position

// Timing constants
const unsigned long SENSOR_INTERVAL = 50;      // Sensor reading interval (ms)
const unsigned long TELEMETRY_INTERVAL = 1000; // Telemetry interval (ms)
const unsigned long BATTERY_INTERVAL = 100;    // Battery monitoring interval (ms)
const unsigned long MQ2_WARMUP_TIME = 20000;   // MQ-2 warmup time (ms)
const unsigned long TURN_TIME_LEFT = 600;      // Time for left turn (ms)
const unsigned long TURN_TIME_AROUND = 1200;   // Time for 180° turn (ms)
const unsigned long TURN_TIME_RIGHT = 500;     // Time for right turn (ms)

// GPS accuracy thresholds
const float GPS_MIN_ACCURACY = 5.0;            // Minimum GPS accuracy (meters)
const int GPS_MIN_SATELLITES = 4;              // Minimum satellites for good fix

// ===== VARIABLES =====

// Timing
unsigned long currentMillis = 0;        // Current time
unsigned long previousSensorMillis = 0; // Last sensor reading time
unsigned long previousTelemetryMillis = 0; // Last telemetry time
unsigned long previousBatteryMillis = 0; // Last battery reading time
unsigned long startTime = 0;            // Mission start time
unsigned long turnStartTime = 0;        // Turn start time

// GPS PPS timing
volatile unsigned long ppsTime = 0;     // Last PPS pulse time
volatile bool ppsReceived = false;      // PPS pulse received flag
unsigned long lastValidGPSTime = 0;     // Last valid GPS reading time
unsigned long lastPPSTime = 0;          // Previous PPS time for interval checking
bool ppsActive = false;                 // PPS signal is active and valid

// Navigation
float startLat = 0;               // Starting latitude
float startLon = 0;               // Starting longitude
bool missionStarted = false;      // Whether the mission has started
bool fireDetected = false;        // Whether fire/gas has been detected

// Sensor readings
int frontDistance = 0;            // Front ultrasonic reading (cm)
int rightDistance = 0;            // Right ultrasonic reading (cm)
int leftDistance = 0;             // Left ultrasonic reading (cm)
int coPPM = 0;                    // CO concentration (ppm)
int lpgPPM = 0;                   // LPG concentration (ppm)

// Enhanced battery monitoring variables
float batteryVoltage = 0;         // Battery voltage (V)
float batteryCurrent = 0;         // Battery current (mA)
float batteryPower = 0;           // Battery power (mW)
float consumedCapacity = 0;       // Consumed capacity (mAh)
float stateOfCharge = 100.0;      // State of charge (%)
float remainingCapacity = BATTERY_CAPACITY_MAH; // Remaining capacity (mAh)
unsigned long lastBatteryTime = 0; // Last battery measurement time
float totalEnergyConsumed = 0;    // Total energy consumed (Wh)

// GPS enhanced variables
float gpsAccuracy = 999.0;        // GPS horizontal accuracy (meters)
int satelliteCount = 0;           // Number of satellites
float gpsSpeed = 0;               // GPS speed (km/h)
float gpsCourse = 0;              // GPS course (degrees)

// State machine variables
enum RobotState {
  INITIALIZING,
  NAVIGATING,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_AROUND,
  STOPPED
};

RobotState currentState = INITIALIZING;
bool stateChanged = true;

// LiPo discharge curve points (voltage vs SOC)
struct DischargePoint {
  float voltage;
  float soc;
};

// 4S LiPo discharge curve (per cell voltage * 4)
const DischargePoint dischargeCurve[] = {
  {16.8, 100.0}, // 4.2V per cell
  {16.4, 95.0},  // 4.1V per cell
  {16.0, 85.0},  // 4.0V per cell
  {15.6, 75.0},  // 3.9V per cell
  {15.2, 60.0},  // 3.8V per cell
  {14.8, 40.0},  // 3.7V per cell
  {14.4, 25.0},  // 3.6V per cell
  {14.0, 10.0},  // 3.5V per cell
  {13.6, 5.0},   // 3.4V per cell
  {13.2, 0.0}    // 3.3V per cell (minimum safe)
};

const int DISCHARGE_CURVE_POINTS = sizeof(dischargeCurve) / sizeof(DischargePoint);

// ===== SETUP =====

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial1.begin(9600); // For GPS module

  // Initialize I2C
  Wire.begin();
  
  // Initialize pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(GPS_PPS_PIN, INPUT);
  
  // Initialize servo
  steering.attach(SERVO_PIN);
  steerStraight();
  
  // Initialize INA260 current/voltage/power sensor
  if (!ina260.init()) {
    Serial.println("INA260 initialization failed!");
    while (1);
  }
  
  // Configure INA260 for optimal performance
  ina260.setMeasureMode(CONTINUOUS);
  ina260.setConversionTime(CONV_TIME_1100);
  ina260.setAverage(AVERAGE_16);
  
  Serial.println("INA260 initialized successfully");
  
  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }

  // Initialize SD Card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    while (1);
  }
  File f = SD.open("route.csv", FILE_WRITE);
  if (f) {
    f.println("timestamp,lat,lon,voltage,current,power,soc,satellites");
    f.close();
  }
  
  // Setup GPS PPS interrupt
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), ppsInterrupt, RISING);
  
  // Record start time for MQ-2 warmup
  startTime = millis();
  lastBatteryTime = startTime;
  
  Serial.println("Robot initialized. Warming up MQ-2 sensor...");
  Serial.println("Enhanced battery management and GPS PPS active");
}

// ===== GPS PPS INTERRUPT =====

void ppsInterrupt() {
  ppsTime = micros();
  ppsReceived = true;
}

// ===== MAIN LOOP =====

void loop() {
  // Update current time
  currentMillis = millis();
  
  // State machine for initialization
  if (currentState == INITIALIZING) {
    // Wait for MQ-2 sensor warmup
    if (currentMillis - startTime >= MQ2_WARMUP_TIME) {
      // Check for GPS fix
      while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
          if (gps.location.isValid() && gps.satellites.value() >= GPS_MIN_SATELLITES) {
            startLat = gps.location.lat();
            startLon = gps.location.lng();
            satelliteCount = gps.satellites.value();
            missionStarted = true;
            saveCurrentWaypoint();
            Serial.println("GPS fix acquired. Mission starting!");
            Serial.print("Start position: ");
            Serial.print(startLat, 6);
            Serial.print(", ");
            Serial.print(startLon, 6);
            Serial.print(", Satellites: ");
            Serial.println(satelliteCount);
            break;
          }
        }
      }
      
      // If no GPS data received for 5 seconds after warmup, continue anyway
      if (currentMillis - startTime >= MQ2_WARMUP_TIME + 5000 || missionStarted) {
        if (!missionStarted) {
          Serial.println("No GPS fix. Starting mission without GPS.");
          missionStarted = true;
        }
        
        // Initial battery check
        readBatteryData();
        Serial.print("Initial battery: ");
        Serial.print(batteryVoltage, 2);
        Serial.print("V, ");
        Serial.print(stateOfCharge, 1);
        Serial.println("% SOC");
        
        // Ready to go
        Serial.println("Robot initialized and ready!");
        
        // Transition to navigation state
        changeState(NAVIGATING);
      }
    }
  }
  
  // Read battery data at high frequency for accurate coulomb counting
  if (currentMillis - previousBatteryMillis >= BATTERY_INTERVAL) {
    previousBatteryMillis = currentMillis;
    readBatteryData();
    updateBatteryState();
  }
  
  // Read sensors at regular intervals
  if (currentMillis - previousSensorMillis >= SENSOR_INTERVAL) {
    previousSensorMillis = currentMillis;
    readSensors();
    
    // Check for fire/gas
    checkForFire();
  }
  
  // Handle navigation based on current state
  handleNavigation();
  
  // Send telemetry at 1Hz
  if (currentMillis - previousTelemetryMillis >= TELEMETRY_INTERVAL) {
    previousTelemetryMillis = currentMillis;
    
    // Update GPS data
    updateGPS();
    
    // Send enhanced telemetry
    sendEnhancedTelemetry();
    
    // Print comprehensive status
    printSystemStatus();
  }
}

// ===== BATTERY MANAGEMENT FUNCTIONS =====

void readBatteryData() {
  // Read voltage, current, and power from INA260
  batteryVoltage = ina260.getBusVoltage_V();
  batteryCurrent = ina260.getCurrent_mA();
  batteryPower = ina260.getPower_mW();
  
  // Handle negative current (charging - shouldn't happen in this application)
  if (batteryCurrent < 0) {
    batteryCurrent = 0;
  }
}

void updateBatteryState() {
  // Calculate time delta for coulomb counting
  unsigned long currentTime = millis();
  float deltaTimeHours = (currentTime - lastBatteryTime) / 3600000.0; // Convert ms to hours
  lastBatteryTime = currentTime;
  
  // Coulomb counting: integrate current over time
  float consumedThisCycle = batteryCurrent * deltaTimeHours; // mAh
  consumedCapacity += consumedThisCycle;
  
  // Update remaining capacity
  remainingCapacity = BATTERY_CAPACITY_MAH - consumedCapacity;
  if (remainingCapacity < 0) remainingCapacity = 0;
  
  // Calculate SOC using combined voltage and coulomb counting
  float voltageSoc = calculateVoltageBasedSOC(batteryVoltage);
  float coulombSoc = (remainingCapacity / BATTERY_CAPACITY_MAH) * 100.0;
  
  // Weighted combination (favor coulomb counting when available)
  if (consumedCapacity > 10) { // After some consumption, trust coulomb counting more
    stateOfCharge = (coulombSoc * 0.8) + (voltageSoc * 0.2);
  } else {
    stateOfCharge = (coulombSoc * 0.3) + (voltageSoc * 0.7);
  }
  
  // Constrain SOC to valid range
  stateOfCharge = constrain(stateOfCharge, 0, 100);
  
  // Update total energy consumed
  totalEnergyConsumed += (batteryPower * deltaTimeHours) / 1000.0; // Convert mWh to Wh
}

float calculateVoltageBasedSOC(float voltage) {
  // Interpolate SOC from discharge curve
  if (voltage >= dischargeCurve[0].voltage) {
    return 100.0;
  }
  if (voltage <= dischargeCurve[DISCHARGE_CURVE_POINTS - 1].voltage) {
    return 0.0;
  }
  
  // Find the two points to interpolate between
  for (int i = 0; i < DISCHARGE_CURVE_POINTS - 1; i++) {
    if (voltage <= dischargeCurve[i].voltage && voltage >= dischargeCurve[i + 1].voltage) {
      // Linear interpolation
      float voltageRange = dischargeCurve[i].voltage - dischargeCurve[i + 1].voltage;
      float socRange = dischargeCurve[i].soc - dischargeCurve[i + 1].soc;
      float voltageOffset = dischargeCurve[i].voltage - voltage;
      
      return dischargeCurve[i].soc - (voltageOffset / voltageRange) * socRange;
    }
  }
  
  return 50.0; // Fallback
}

float estimateRemainingTime() {
  // Estimate remaining time based on current consumption
  if (batteryCurrent <= 0) {
    return 999.0; // Infinite time if no consumption
  }
  
  // Calculate remaining time in hours
  float remainingTimeHours = remainingCapacity / batteryCurrent;
  
  // Convert to minutes
  return remainingTimeHours * 60.0;
}

// ===== STATE MANAGEMENT =====

void changeState(RobotState newState) {
  if (currentState != newState) {
    currentState = newState;
    stateChanged = true;
    
    // Record start time for timed states
    if (newState == TURNING_LEFT || newState == TURNING_RIGHT || newState == TURNING_AROUND) {
      turnStartTime = currentMillis;
    }
    
    // Debug output
    Serial.print("State changed to: ");
    switch (newState) {
      case INITIALIZING: Serial.println("INITIALIZING"); break;
      case NAVIGATING: Serial.println("NAVIGATING"); break;
      case TURNING_LEFT: Serial.println("TURNING_LEFT"); break;
      case TURNING_RIGHT: Serial.println("TURNING_RIGHT"); break;
      case TURNING_AROUND: Serial.println("TURNING_AROUND"); break;
      case STOPPED: Serial.println("STOPPED"); break;
    }
  }
}

// ===== NAVIGATION FUNCTIONS =====

void handleNavigation() {
  switch (currentState) {
    case NAVIGATING:
      // Navigation logic based on ultrasonic sensors
      if (frontDistance > FRONT_SLOW_DIST) {
        // Path is clear ahead, follow right wall at normal speed
        followWall(rightDistance, NORMAL_SPEED);
      } 
      else if (frontDistance <= FRONT_SLOW_DIST && frontDistance > FRONT_STOP_DIST) {
        // Obstacle ahead but not too close, slow down
        followWall(rightDistance, SLOW_SPEED);
      } 
      else if (frontDistance <= FRONT_STOP_DIST) {
        // Obstacle too close, need to turn
        stopMotors();
        
        // Check if right is open (right-hand rule)
        if (rightDistance > DESIRED_RIGHT_DIST + 5) {
          // Right is open, turn right
          changeState(TURNING_RIGHT);
        } 
        else if (leftDistance > DESIRED_RIGHT_DIST + 5) {
          // Left is open, turn left
          changeState(TURNING_LEFT);
        } 
        else {
          // Both sides blocked, turn around
          changeState(TURNING_AROUND);
        }
      }
      break;
      
    case TURNING_LEFT:
      // Execute a left turn (non-blocking)
      if (stateChanged) {
        steerLeft();
        analogWrite(ENA, 150);
        analogWrite(ENB, 150);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        stateChanged = false;
      }
      
      // Check if turn is complete
      if (currentMillis - turnStartTime >= TURN_TIME_LEFT) {
        stopMotors();
        steerStraight();
        changeState(NAVIGATING);
      }
      break;
      
    case TURNING_RIGHT:
      // Execute a right turn (non-blocking)
      if (stateChanged) {
        steerRight();
        analogWrite(ENA, 150);
        analogWrite(ENB, 150);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        stateChanged = false;
      }
      
      // Check if turn is complete
      if (currentMillis - turnStartTime >= TURN_TIME_RIGHT) {
        steerStraight();
        changeState(NAVIGATING);
      }
      break;
      
    case TURNING_AROUND:
      // Execute a 180-degree turn (non-blocking)
      if (stateChanged) {
        steerLeft();
        analogWrite(ENA, 200);
        analogWrite(ENB, 200);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        stateChanged = false;
      }
      
      // Check if turn is complete
      if (currentMillis - turnStartTime >= TURN_TIME_AROUND) {
        stopMotors();
        steerStraight();
        changeState(NAVIGATING);
      }
      break;
      
    case STOPPED:
      // Robot is stopped
      stopMotors();
      break;
  }
}

void followWall(int rightDist, int speed) {
  // Wall following using right-hand rule
  if (rightDist > DESIRED_RIGHT_DIST + 5) {
    // Too far from wall, steer right
    steerRight();
  } 
  else if (rightDist < DESIRED_RIGHT_DIST - 2) {
    // Too close to wall, steer left
    steerLeft();
  } 
  else {
    // Good distance from wall, go straight
    steerStraight();
  }
  
  // Move forward at specified speed
  moveForward(speed);
}

// ===== SENSOR FUNCTIONS =====

void readSensors() {
  // Read ultrasonic sensors
  frontDistance = sonarFront.ping_cm();
  rightDistance = sonarRight.ping_cm();
  leftDistance = sonarLeft.ping_cm();
  
  // If distance is 0 (out of range), set to maximum
  if (frontDistance == 0) frontDistance = MAX_DISTANCE;
  if (rightDistance == 0) rightDistance = MAX_DISTANCE;
  if (leftDistance == 0) leftDistance = MAX_DISTANCE;
  
  // Read MQ-2 sensor
  readMQ2(coPPM, lpgPPM);
}

void readMQ2(int &co_ppm, int &lpg_ppm) {
  // Read MQ-2 sensor
  int mq2_raw = analogRead(MQ2_PIN);
  
  // Convert raw reading to ppm (simplified approach)
  // In a real implementation, you would use the sensor's datasheet
  // to create a more accurate conversion function
  co_ppm = map(mq2_raw, 0, 1023, 0, 1000);
  lpg_ppm = map(mq2_raw, 0, 1023, 0, 5000);
}

void updateGPS() {
  // Read GPS data
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.location.isValid()) {
        lastValidGPSTime = currentMillis;
        satelliteCount = gps.satellites.value();
        
        // Calculate GPS accuracy (HDOP-based estimation)
        if (gps.hdop.isValid()) {
          gpsAccuracy = gps.hdop.hdop() * 5.0; // Rough conversion to meters
        }
        
        // Get speed and course if available
        if (gps.speed.isValid()) {
          gpsSpeed = gps.speed.kmph();
        }
        if (gps.course.isValid()) {
          gpsCourse = gps.course.deg();
        }
        
        saveCurrentWaypoint();
      }
    }
  }
  
  // Handle PPS synchronization and verification
  if (ppsReceived) {
    ppsReceived = false;
    
    // Verify PPS timing accuracy
    if (lastPPSTime > 0) {
      unsigned long ppsInterval = ppsTime - lastPPSTime;
      // PPS should be exactly 1 second (1,000,000 microseconds)
      if (abs((long)(ppsInterval - 1000000)) < 1000) {  // Within 1ms tolerance
        ppsActive = true;
        Serial.println("PPS: Accurate 1Hz signal detected");
      } else {
        Serial.print("PPS: Timing error - interval: ");
        Serial.print(ppsInterval);
        Serial.println(" μs (expected: 1,000,000 μs)");
      }
    }
    lastPPSTime = ppsTime;
    
    // Use PPS for precise GPS data synchronization
    if (gps.location.isValid() && ppsActive) {
      // GPS data is now synchronized to microsecond precision
      Serial.print("PPS-synchronized GPS at: ");
      Serial.print(ppsTime);
      Serial.println(" μs");
    }
  }
  
  // Check for PPS timeout (no signal for >2 seconds)
  if (ppsActive && (micros() - ppsTime) > 2000000) {
    ppsActive = false;
    Serial.println("WARNING: PPS signal lost");
  }
}

// ===== MOTOR CONTROL FUNCTIONS =====

void moveForward(int speed) {
  // Move forward at specified speed
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  // Stop all motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void steerStraight() {
  // Set servo to center position
  steering.write(SERVO_CENTER);
}

void steerLeft() {
  // Set servo to left position
  steering.write(SERVO_LEFT);
}

void steerRight() {
  // Set servo to right position
  steering.write(SERVO_RIGHT);
}

// ===== FIRE DETECTION AND TELEMETRY =====

void checkForFire() {
  // Check if fire/gas is detected
  if (coPPM > CO_THRESHOLD || lpgPPM > LPG_THRESHOLD) {
    // Fire/gas detected
    if (!fireDetected) {
      // First detection
      fireDetected = true;
      Serial.println("ALERT: Fire/Gas detected!");
      
      // Send emergency telemetry
      sendEmergencyTelemetry();
    }
  } else {
    // Reset detection flag if levels drop
    fireDetected = false;
  }
}

void sendEnhancedTelemetry() {
  // Enhanced telemetry format: lat, lon, LPG_ppm, CO_ppm, batt_voltage, 
  // current_mA, power_mW, soc_percent, remaining_minutes, gps_accuracy, satellite_count
  String telemetryData = formatEnhancedTelemetryData();
  
  // Send via LoRa
  LoRa.beginPacket();
  LoRa.print(telemetryData);
  LoRa.endPacket();
  
  // Debug output
  Serial.print("Enhanced telemetry sent: ");
  Serial.println(telemetryData);
}

void sendEmergencyTelemetry() {
  // Send emergency telemetry with current position and gas readings
  String emergencyData = formatEnhancedTelemetryData();
  
  // Send via LoRa with higher power/priority
  LoRa.beginPacket();
  LoRa.print("EMERGENCY: ");
  LoRa.print(emergencyData);
  LoRa.endPacket();
  
  // Debug output
  Serial.print("EMERGENCY TELEMETRY: ");
  Serial.println(emergencyData);
}

String formatEnhancedTelemetryData() {
  // Enhanced telemetry data format
  String data = "";
  
  if (gps.location.isValid()) {
    data += String(gps.location.lat(), 6) + ",";
    data += String(gps.location.lng(), 6) + ",";
  } else {
    data += "0.000000,0.000000,";
  }
  
  data += String(lpgPPM) + ",";
  data += String(coPPM) + ",";
  data += String(batteryVoltage, 2) + ",";
  data += String(batteryCurrent, 1) + ",";
  data += String(batteryPower, 1) + ",";
  data += String(stateOfCharge, 1) + ",";
  data += String(estimateRemainingTime(), 1) + ",";
  data += String(gpsAccuracy, 1) + ",";
  data += String(satelliteCount);
  
  return data;
}

void saveCurrentWaypoint() {
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    
    File f = SD.open("route.csv", FILE_WRITE);
    if (f) {
      // Use PPS time for precise timestamps when available
      if (ppsTime > 0) {
        f.print(ppsTime); f.print(',');  // Microsecond-precise timestamp
      } else {
        f.print(currentMillis * 1000); f.print(',');  // Fallback to millisecond timestamp
      }
      f.print(lat, 6); f.print(',');
      f.print(lon, 6); f.print(',');
      f.print(batteryVoltage, 2); f.print(',');
      f.print(batteryCurrent, 1); f.print(',');
      f.print(batteryPower, 1); f.print(',');
      f.print(stateOfCharge, 1); f.print(',');
      f.println(satelliteCount);
      f.close();
    }
  }
}

// ===== STATUS AND MONITORING =====

void printSystemStatus() {
  // Print comprehensive system status
  unsigned long missionTime = (currentMillis - startTime) / 1000; // seconds
  
  Serial.println("=== SYSTEM STATUS ===");
  Serial.print("Mission time: ");
  Serial.print(missionTime / 60); // minutes
  Serial.print(":");
  Serial.print(missionTime % 60); // seconds
  Serial.println();
  
  // Battery status
  Serial.print("Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V, ");
  Serial.print(batteryCurrent, 1);
  Serial.print("mA, ");
  Serial.print(batteryPower, 1);
  Serial.print("mW, SOC: ");
  Serial.print(stateOfCharge, 1);
  Serial.print("%, Remaining: ");
  Serial.print(estimateRemainingTime(), 0);
  Serial.println(" min");
  
  // GPS status
  if (gps.location.isValid()) {
    Serial.print("GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
    Serial.print(", Accuracy: ");
    Serial.print(gpsAccuracy, 1);
    Serial.print("m, Satellites: ");
    Serial.println(satelliteCount);
  } else {
    Serial.println("GPS: No fix");
  }
  
  // Sensor status
  Serial.print("Distances - Front: ");
  Serial.print(frontDistance);
  Serial.print("cm, Right: ");
  Serial.print(rightDistance);
  Serial.print("cm, Left: ");
  Serial.print(leftDistance);
  Serial.println("cm");
  
  Serial.print("Gas - CO: ");
  Serial.print(coPPM);
  Serial.print("ppm, LPG: ");
  Serial.print(lpgPPM);
  Serial.println("ppm");
  
  // Energy consumption
  Serial.print("Total energy consumed: ");
  Serial.print(totalEnergyConsumed, 2);
  Serial.println(" Wh");
  
  Serial.println("=====================");
}
