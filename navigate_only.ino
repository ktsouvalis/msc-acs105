// Ignore this first bit but don't delete or change it
// this bit is usually hidden inside 
// #import <NewPing.h>
// THe class NewPing is for the tinkercad only
// The final code will have only the import <NewPing.h>
class NewPing {
  public:
    NewPing(int TRIGGER_PIN, int ECHO_PIN, int MAX_DISTANCE ){
      trigPin=TRIGGER_PIN;
      echoPin=ECHO_PIN;
      maxDistance=MAX_DISTANCE;

      pinMode(trigPin,OUTPUT);
      pinMode(echoPin,INPUT);
    }

    int ping_cm(){
      // Clears the trigPin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microsecond
      long duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      int distance =  duration*0.034/2;
      // Checks out of range
      if (distance>maxDistance){
        distance=0;
      }
      return(distance);
    }

  private:
    int trigPin;
    int echoPin;
    int maxDistance;
};
// this is the bit you can modify

//#include <NewPing.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <LoRa.h> // SX1278 LoRa module

// Ultrasound Pins
#define TRIG_FRONT 2
#define ECHO_FRONT 3
#define TRIG_RIGHT 4
#define ECHO_RIGHT 5
//#define TRIG_LEFT 6
//#define ECHO_LEFT 7
#define MAX_DISTANCE 200

NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
//NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);

// Motors
#define ENA 9   // Left motor PWM
#define IN1 7
#define IN2 8
#define ENB 6  // Right motor PWM
#define IN3 12
#define IN4 13

// Servo
Servo steering;
#define SERVO_PIN 11

// GPS
TinyGPSPlus gps;


// Battery voltage sense pin
#define BATT_PIN A2

// MQ-2 sensor pin (add if missing)
#define MQ2_PIN A0

// Thresholds
const int CO_THRESHOLD = 50;    // ppm
const int LPG_THRESHOLD = 2100; // ppm

// Calibration values for battery voltage divider
const float BATT_R1 = 10000.0; // 10k
const float BATT_R2 = 2000.0;  // 2k
const float ADC_REF = 5.0;     // V
const float ADC_RES = 1023.0;

// Constants
const int NORMAL_SPEED = 200;
const int SLOW_SPEED = 50;
const int DESIRED_RIGHT_DIST = 20;
const int FRONT_SLOW_DIST = 50;
const int FRONT_STOP_DIST = 20;
unsigned long lastGPSTime = 0; // For 1Hz sampling

// --- Route saving variables ---
#define MAX_WAYPOINTS 500
double route_lat[MAX_WAYPOINTS];
double route_lon[MAX_WAYPOINTS];
int route_size = 0;

// Minimum distance (meters) between saved waypoints to avoid duplicates
#define WAYPOINT_MIN_DIST 2.0

// Helper: Calculate distance between two GPS points (Haversine, simplified for short distances)
float gpsDistance(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0; // Earth radius in meters
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// Helper: Save current GPS position as a waypoint if far enough from last
void saveCurrentWaypoint() {
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  if (route_size == 0 || gpsDistance(route_lat[route_size-1], route_lon[route_size-1], lat, lon) > WAYPOINT_MIN_DIST) {
    if (route_size < MAX_WAYPOINTS) {
      route_lat[route_size] = lat;
      route_lon[route_size] = lon;
      route_size++;
    }
  }
}

// Helper: Calculate bearing from current position to target waypoint
float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) -
            sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float brng = atan2(y, x);
  brng = degrees(brng);
  if (brng < 0) brng += 360;
  return brng;
}

// Helper: Steer toward a bearing (simple version)
void steerToBearing(float bearing) {
  // 0 = North, 90 = East, 180 = South, 270 = West
  // Assume 90 is straight, 60 is right, 120 is left
  if (bearing > 315 || bearing < 45) {
    steerStraight();
  } else if (bearing >= 45 && bearing < 135) {
    steerRight();
  } else if (bearing >= 135 && bearing < 225) {
    steerStraight(); // Could implement reverse if needed
  } else {
    steerLeft();
  }
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  steering.attach(SERVO_PIN);
  Serial.begin(9600);
  LoRa.begin(433E6); // Initialize LoRa at 433MHz
}

float readBatteryVoltage() {
  int raw = analogRead(BATT_PIN);
  float vout = (raw * ADC_REF) / ADC_RES;
  float vin = vout / (BATT_R2 / (BATT_R1 + BATT_R2));
  return vin;
}

void readMQ2(int &co_ppm, int &lpg_ppm) {
  int mq2_raw = analogRead(MQ2_PIN);
  co_ppm = map(mq2_raw, 0, 1023, 0, 1000);   // simplified approach: not accurate, just for demonstration
  lpg_ppm = map(mq2_raw, 0, 1023, 0, 5000);  // simplified approach: not accurate, just for demonstration
}

void sendTelemetry(int lpg_ppm, int co_ppm, float batt_voltage) {
  String msg = String(gps.location.lat(), 6) + "," +
               String(gps.location.lng(), 6) + "," +
               String(lpg_ppm) + "," +
               String(co_ppm) + "," +
               String(batt_voltage, 2);
  LoRa.println(msg);
}

void followWall(int rightDist, int speed) {
  if (rightDist > DESIRED_RIGHT_DIST + 2) { // Too far from wall with tolerance
    steerRight();
  } else if (rightDist < DESIRED_RIGHT_DIST - 2) { // Too close to wall with tolerance
    steerLeft();
  } else {
    steerStraight();
  }
  moveForward(speed);
}

void moveForward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void steerStraight() {
  steering.write(90);
}

void steerLeft() {
  steering.write(120);
}

void steerRight() {
  steering.write(60);
}

void turnLeft() {
  // Slight reverse and left
  steerLeft();
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(600);  // delay for servo to move, xreiazetai tuning
  stopMotors();
}

// Add a flag to enable return-to-home mode
bool returning = false;

void loop() {
  int frontDist = sonarFront.ping_cm(); //find front distance
  int rightDist = sonarRight.ping_cm(); //find right distance

  // --- Battery and position check for return-to-home ---
  float batt_voltage = readBatteryVoltage();
  float min_voltage = 6.0;
  float max_voltage = 8.4;
  float percent = (batt_voltage - min_voltage) / (max_voltage - min_voltage);
  percent = constrain(percent, 0, 1);

  // Check if at return point (last waypoint)
  bool at_return_point = false;
  if (route_size > 0 && gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    float dist_to_return = gpsDistance(lat, lon, route_lat[route_size-1], route_lon[route_size-1]);
    if (dist_to_return < WAYPOINT_MIN_DIST) {
      at_return_point = true;
    }
  }

  // Set returning true if battery ≤15% or at return point (last waypoint)
  if (!returning && (percent <= 0.15 || at_return_point)) {
    returning = true;
  }

  // --- Return-to-home logic ---
  static int return_idx = -1;
  if (returning) {
    // If just started returning, set index to last waypoint
    if (return_idx == -1) return_idx = route_size - 1;
    if (return_idx >= 0 && gps.location.isValid()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      float dist = gpsDistance(lat, lon, route_lat[return_idx], route_lon[return_idx]);
      float bearing = gpsBearing(lat, lon, route_lat[return_idx], route_lon[return_idx]);
      steerToBearing(bearing);
      moveForward(NORMAL_SPEED);
      if (dist < WAYPOINT_MIN_DIST) {
        return_idx--;
      }
    } else {
      stopMotors(); // Arrived home or no waypoints
    }
  } else {
    // --- Normal navigation logic: wall following and obstacle avoidance ---
    if (frontDist > FRONT_SLOW_DIST) {
      followWall(rightDist, NORMAL_SPEED);
    } else if (frontDist <= FRONT_SLOW_DIST && frontDist > FRONT_STOP_DIST) {
      followWall(rightDist, SLOW_SPEED);
    } else if (frontDist <= FRONT_STOP_DIST) {
      if (rightDist < DESIRED_RIGHT_DIST + 2) { // Right is blocked, turn left with tolerance
        turnLeft();
      } else {
        followWall(rightDist, SLOW_SPEED); // Right is open, follow wall slowly (turn into opening)
      }
    }
  }

  // === 1Hz GPS sampling and telemetry ===
  unsigned long now = millis();
  if (now - lastGPSTime >= 1000) {
    lastGPSTime = now;

    // Read sensors
    int co_ppm, lpg_ppm;
    readMQ2(co_ppm, lpg_ppm);

    // Read GPS
    while (Serial1.available()) gps.encode(Serial1.read());
    // Send telemetry every second
    sendTelemetry(lpg_ppm, co_ppm, batt_voltage);

    // Εκτίμηση χρόνου λειτουργίας (απλή προσέγγιση)
    Serial.print("Estimated battery %: ");
    Serial.println(percent * 100, 1);
  }

  // === Fire/gas detection and emergency telemetry ===
  int co_ppm, lpg_ppm;
  readMQ2(co_ppm, lpg_ppm);
  float batt_voltage = readBatteryVoltage();
  if (co_ppm > CO_THRESHOLD || lpg_ppm > LPG_THRESHOLD) {
    // Read GPS for current position
    while (Serial1.available()) gps.encode(Serial1.read());

    Serial.print("CO: "); Serial.print(co_ppm);
    Serial.print(" ppm, LPG: "); Serial.print(lpg_ppm);
    Serial.print(" ppm, Battery: "); Serial.println(batt_voltage, 2);

    // Send emergency telemetry (same format)
    sendTelemetry(lpg_ppm, co_ppm, batt_voltage);
    delay(500); // avoid spamming
  }

  // Save route while navigating (not returning)
  static unsigned long lastSave = 0;
  if (!returning && millis() - lastSave > 1000) {
    lastSave = millis();
    if (gps.location.isValid()) {
      saveCurrentWaypoint();
    }
  }

  delay(100); // Main loop delay for sensor stability
}


