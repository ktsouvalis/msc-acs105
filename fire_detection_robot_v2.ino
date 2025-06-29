/*
 * Unmanned Fire Detection Robot - Version 2 (Optimized)
 * 
 * This version uses non-blocking timing techniques instead of delay() functions
 * to improve responsiveness and allow concurrent operations.
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - 3x HC-SR04 Ultrasonic sensors (front, 45° left, 45° right)
 * - L298N Motor Driver for DC motors
 * - Servo motor for Ackermann steering
 * - MQ-2 Gas sensor for fire/gas detection
 * - NEO-6M GPS module
 * - SX1278 LoRa module for telemetry
 * - TATTU 2300mAh 14.8V LiPo Battery
 */

#include <NewPing.h>         // Ultrasonic sensors
#include <Servo.h>           // Steering servo
#include <TinyGPS++.h>       // GPS module
#include <LoRa.h>            // SX1278 LoRa module
#include <SD.h>


// ===== PIN DEFINITIONS =====

// --- SD Card Pins ---
#define SD_CS_PIN 10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK 13

// Ultrasonic Sensors
#define TRIG_FRONT 2
#define ECHO_FRONT 3
#define TRIG_RIGHT 4
#define ECHO_RIGHT 5
#define TRIG_LEFT 6
#define ECHO_LEFT 7
#define MAX_DISTANCE 200 // Maximum distance for ultrasonic sensors (cm)

// Motor Driver (L298N)
#define ENA 9   // Left motor PWM
#define IN1 22  // Changed from 7 to avoid conflict with TRIG_LEFT
#define IN2 23  // Changed from 8 to avoid conflict with ECHO_LEFT
#define ENB 8   // Right motor PWM (changed from 6 to avoid conflict)
#define IN3 24
#define IN4 25

// Servo
#define SERVO_PIN 44

// Sensors
#define MQ2_PIN A0      // MQ-2 Gas sensor
#define BATT_PIN A2     // Battery voltage monitoring

// LoRa Module (SX1278)
#define LORA_SS 53      // NSS pin
#define LORA_RST 49     // Reset pin
#define LORA_DIO0 48    // DIO0 pin

// ===== OBJECTS =====

// Ultrasonic sensors
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);

// Servo for steering
Servo steering;

// GPS
TinyGPSPlus gps;

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

// Calibration values for battery voltage divider
const float BATT_R1 = 10000.0;    // 10k resistor
const float BATT_R2 = 2000.0;     // 2k resistor
const float ADC_REF = 5.0;        // Reference voltage (V)
const float ADC_RES = 1023.0;     // ADC resolution

// Servo angles
const int SERVO_CENTER = 90;      // Straight position
const int SERVO_LEFT = 120;       // Left turn position
const int SERVO_RIGHT = 60;       // Right turn position

// Timing constants
const unsigned long SENSOR_INTERVAL = 50;      // Sensor reading interval (ms)
const unsigned long TELEMETRY_INTERVAL = 1000; // Telemetry interval (ms)
const unsigned long MQ2_WARMUP_TIME = 20000;   // MQ-2 warmup time (ms)
const unsigned long TURN_TIME_LEFT = 600;      // Time for left turn (ms)
const unsigned long TURN_TIME_AROUND = 1200;   // Time for 180° turn (ms)
const unsigned long TURN_TIME_RIGHT = 500;     // Time for right turn (ms)

// ===== VARIABLES =====

// Timing
unsigned long currentMillis = 0;        // Current time
unsigned long previousSensorMillis = 0; // Last sensor reading time
unsigned long previousTelemetryMillis = 0; // Last telemetry time
unsigned long startTime = 0;            // Mission start time
unsigned long turnStartTime = 0;        // Turn start time

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
float batteryVoltage = 0;         // Battery voltage (V)

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

// ===== SETUP =====

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial1.begin(9600); // For GPS module

  
  // Initialize pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Initialize servo
  steering.attach(SERVO_PIN);
  steerStraight();
  
  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }

  // Initialze SD Card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    while (1);
  }
  File f = SD.open("route.csv", FILE_WRITE);
  if (f) {
    f.println("lat,lon");
    f.close();
  }
  
  // Record start time for MQ-2 warmup
  startTime = millis();
  
  Serial.println("Robot initialized. Warming up MQ-2 sensor...");
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
          if (gps.location.isValid()) {
            startLat = gps.location.lat();
            startLon = gps.location.lng();
            missionStarted = true;
            saveCurrentWaypoint();
            Serial.println("GPS fix acquired. Mission starting!");
            Serial.print("Start position: ");
            Serial.print(startLat, 6);
            Serial.print(", ");
            Serial.println(startLon, 6);
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
        batteryVoltage = readBatteryVoltage();
        Serial.print("Battery voltage: ");
        Serial.println(batteryVoltage, 2);
        
        // Ready to go
        Serial.println("Robot initialized and ready!");
        
        // Transition to navigation state
        changeState(NAVIGATING);
      }
    }
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

    // Estimate remaining battery life
    estimateBatteryLife();
    
    // Send regular telemetry
    sendTelemetry();
    
    // Print mission time
    unsigned long missionTime = (currentMillis - startTime) / 1000; // seconds
    Serial.print("Mission time: ");
    Serial.print(missionTime / 60); // minutes
    Serial.print(":");
    Serial.print(missionTime % 60); // seconds
    Serial.println();
  }
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
  
  // Read battery voltage
  batteryVoltage = readBatteryVoltage();
  
  // Read MQ-2 sensor
  readMQ2(coPPM, lpgPPM);
  
  // Debug output (less frequent to reduce serial traffic)
  if (currentMillis - previousTelemetryMillis >= TELEMETRY_INTERVAL) {
    Serial.print("Distances - Front: ");
    Serial.print(frontDistance);
    Serial.print(" cm, Right: ");
    Serial.print(rightDistance);
    Serial.print(" cm, Left: ");
    Serial.print(leftDistance);
    Serial.print(" cm, Heading: ");
    Serial.print(heading);
    Serial.println("°");
  }
}

void readMQ2(int &co_ppm, int &lpg_ppm) {
  // Read MQ-2 sensor
  int mq2_raw = analogRead(MQ2_PIN);
  
  // Convert raw reading to ppm (simplified approach)
  // In a real implementation, you would use the sensor's datasheet
  // to create a more accurate conversion function
  co_ppm = map(mq2_raw, 0, 1023, 0, 1000);
  lpg_ppm = map(mq2_raw, 0, 1023, 0, 5000);
  
  // Debug output (less frequent to reduce serial traffic)
  if (currentMillis - previousTelemetryMillis >= TELEMETRY_INTERVAL) {
    Serial.print("Gas - CO: ");
    Serial.print(co_ppm);
    Serial.print(" ppm, LPG: ");
    Serial.print(lpg_ppm);
    Serial.println(" ppm");
  }
}

float readBatteryVoltage() {
  // Read battery voltage through voltage divider
  int raw = analogRead(BATT_PIN);
  float vout = (raw * ADC_REF) / ADC_RES;
  float vin = vout / (BATT_R2 / (BATT_R1 + BATT_R2));
  return vin;
}

void updateGPS() {
  // Read GPS data
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  
  // Debug output if valid position
  if (gps.location.isValid()) {
    Serial.print("GPS - Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", Lon: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print(", Satellites: ");
    Serial.println(gps.satellites.value());
    saveCurrentWaypoint();
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
      
      // Flash LED or sound buzzer if available
      // (not implemented in this code)
    }
  } else {
    // Reset detection flag if levels drop
    fireDetected = false;
  }
}

void sendTelemetry() {
  // Format: lat, lon, LPG_ppm, CO_ppm, batt_voltage
  String telemetryData = formatTelemetryData();
  
  // Send via LoRa
  LoRa.beginPacket();
  LoRa.print(telemetryData);
  LoRa.endPacket();
  
  // Debug output
  Serial.print("Telemetry sent: ");
  Serial.println(telemetryData);
}

void sendEmergencyTelemetry() {
  // Send emergency telemetry with current position and gas readings
  String emergencyData = formatTelemetryData();
  
  // Send via LoRa with higher power/priority
  LoRa.beginPacket();
  LoRa.print("EMERGENCY: ");
  LoRa.print(emergencyData);
  LoRa.endPacket();
  
  // Debug output
  Serial.print("EMERGENCY TELEMETRY: ");
  Serial.println(emergencyData);
}

String formatTelemetryData() {
  // Format telemetry data according to specification
  // lat, lon, LPG_ppm, CO_ppm, batt_voltage
  String data = "";
  
  if (gps.location.isValid()) {
    data += String(gps.location.lat(), 6) + ",";
    data += String(gps.location.lng(), 6) + ",";
  } else {
    data += "0.000000,0.000000,";
  }
  
  data += String(lpgPPM) + ",";
  data += String(coPPM) + ",";
  data += String(batteryVoltage, 2);
  
  return data;
}

void saveCurrentWaypoint() {
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  static double lastLat = 0, lastLon = 0;
  if (gpsDistance(lastLat, lastLon, lat, lon) > WAYPOINT_MIN_DIST) {
    lastLat = lat; lastLon = lon;
    File f = SD.open("route.csv", FILE_WRITE);
    if (f) {
      f.print(lat, 6); f.print(','); f.println(lon, 6);
      f.close();
    }
  }
}

// ===== BATTERY MANAGEMENT =====

void estimateBatteryLife() {
  // Estimate remaining battery life
  // TATTU 2300mAh 14.8V LiPo Battery
  
  // Battery voltage ranges
  const float MIN_VOLTAGE = 13.2; // Minimum safe voltage (3.3V per cell)
  const float MAX_VOLTAGE = 16.8; // Maximum voltage (4.2V per cell)
  
  // Calculate percentage
  float percentage = (batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0;
  percentage = constrain(percentage, 0, 100);
  
  // Estimate remaining time (simplified)
  // Assuming linear discharge and 4.2 hours total runtime
  float remainingMinutes = (percentage / 100.0) * 252; // 4.2 hours = 252 minutes
  
  // Debug output
  Serial.print("Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V (");
  Serial.print(percentage, 1);
  Serial.print("%), Est. remaining: ");
  Serial.print(int(remainingMinutes / 60));
  Serial.print("h ");
  Serial.print(int(remainingMinutes) % 60);
  Serial.println("m");
  
  // Low battery warning
  if (percentage < 20) {
    Serial.println("WARNING: Low battery!");
  }
}
