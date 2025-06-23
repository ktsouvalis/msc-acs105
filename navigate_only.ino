#include <Servo.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <SD.h>

// --- SD Card Pins ---
#define SD_CS_PIN 10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK 13

// Ultrasound Pins
#define TRIG_FRONT 2
#define ECHO_FRONT 3
#define TRIG_RIGHT 4
#define ECHO_RIGHT 5
#define MAX_DISTANCE 200

class NewPing {
  public:
    NewPing(int TRIGGER_PIN, int ECHO_PIN, int MAX_DISTANCE) {
      trigPin = TRIGGER_PIN;
      echoPin = ECHO_PIN;
      maxDistance = MAX_DISTANCE;
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    int ping_cm() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      long duration = pulseIn(echoPin, HIGH);
      int distance = duration * 0.034 / 2;
      return (distance > maxDistance) ? 0 : distance;
    }

  private:
    int trigPin, echoPin, maxDistance;
};

NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

// Motors
#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 6
#define IN3 12
#define IN4 13

Servo steering;
#define SERVO_PIN 11

TinyGPSPlus gps;

#define BATT_PIN A2
#define MQ2_PIN A0

const int CO_THRESHOLD = 50;
const int LPG_THRESHOLD = 2100;

const float BATT_R1 = 10000.0;
const float BATT_R2 = 2000.0;
const float ADC_REF = 5.0;
const float ADC_RES = 1023.0;

const int NORMAL_SPEED = 200;
const int SLOW_SPEED = 50;
const int DESIRED_RIGHT_DIST = 20;
const int FRONT_SLOW_DIST = 50;
const int FRONT_STOP_DIST = 20;
unsigned long lastGPSTime = 0;

#define WAYPOINT_MIN_DIST 2.0

float gpsDistance(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float brng = atan2(y, x);
  brng = degrees(brng);
  if (brng < 0) brng += 360;
  return brng;
}

void steerToBearing(float bearing) {
  if (bearing > 315 || bearing < 45) steerStraight();
  else if (bearing < 135) steerRight();
  else if (bearing < 225) steerStraight();
  else steerLeft();
}

bool returning = false;
File returnFile;
long returnFilePosition = -1;

bool readLastWaypoint(double &lat, double &lon) {
  if (!returnFile) return false;

  String lastLine = "";
  while (returnFilePosition > 0) {
    returnFilePosition--;
    returnFile.seek(returnFilePosition);
    char c = returnFile.read();
    if (c == '\n' && lastLine.length() > 0) break;
    if (c != '\n' && c != '\r') lastLine = String(c) + lastLine;
  }

  if (lastLine.length() > 0 && lastLine.indexOf(',') > 0) {
    int comma = lastLine.indexOf(',');
    lat = lastLine.substring(0, comma).toFloat();
    lon = lastLine.substring(comma + 1).toFloat();
    return true;
  }

  return false;
}

void startReturnMode() {
  returnFile = SD.open("route.csv", FILE_READ);
  if (!returnFile) {
    Serial.println("Failed to open route.csv for return mode");
    returning = false;
    return;
  }
  returnFile.seek(returnFile.size());
  returnFilePosition = returnFile.position();
  returning = true;
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  steering.attach(SERVO_PIN);
  Serial.begin(9600);
  LoRa.begin(433E6);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    while (1);
  }
  File f = SD.open("route.csv", FILE_WRITE);
  if (f) {
    f.println("lat,lon");
    f.close();
  }
}

float readBatteryVoltage() {
  int raw = analogRead(BATT_PIN);
  float vout = (raw * ADC_REF) / ADC_RES;
  return vout / (BATT_R2 / (BATT_R1 + BATT_R2));
}

void readMQ2(int &co_ppm, int &lpg_ppm) {
  int mq2_raw = analogRead(MQ2_PIN);
  co_ppm = map(mq2_raw, 0, 1023, 0, 1000);
  lpg_ppm = map(mq2_raw, 0, 1023, 0, 5000);
}

void sendTelemetry(int lpg_ppm, int co_ppm, float batt_voltage) {
  String msg = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," +
               String(lpg_ppm) + "," + String(co_ppm) + "," + String(batt_voltage, 2);
  LoRa.println(msg);
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

void followWall(int rightDist, int speed) {
  if (rightDist > DESIRED_RIGHT_DIST + 2) steerRight();
  else if (rightDist < DESIRED_RIGHT_DIST - 2) steerLeft();
  else steerStraight();
  moveForward(speed);
}

void moveForward(int speed) {
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void steerStraight() { steering.write(90); }
void steerLeft() { steering.write(120); }
void steerRight() { steering.write(60); }

void turnLeft() {
  steerLeft();
  analogWrite(ENA, 150); analogWrite(ENB, 150);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(600);
  stopMotors();
}

void loop() {
  int frontDist = sonarFront.ping_cm();
  int rightDist = sonarRight.ping_cm();
  float batt_voltage = readBatteryVoltage();
  float percent = constrain((batt_voltage - 6.0) / (8.4 - 6.0), 0, 1);

  if (!returning && percent <= 0.15) {
    startReturnMode();
  }

  if (returning && gps.location.isValid()) {
    static double tgtLat = 0, tgtLon = 0;
    static bool haveTarget = false;

    if (!haveTarget) {
      if (!readLastWaypoint(tgtLat, tgtLon)) {
        stopMotors();
        returnFile.close();
        return;
      }
      haveTarget = true;
    }

    double lat = gps.location.lat();
    double lon = gps.location.lng();
    float dist = gpsDistance(lat, lon, tgtLat, tgtLon);
    float bearing = gpsBearing(lat, lon, tgtLat, tgtLon);
    steerToBearing(bearing);
    moveForward(NORMAL_SPEED);

    if (dist < WAYPOINT_MIN_DIST) {
      haveTarget = false;
    }
  } else if (!returning) {
    if (frontDist > FRONT_SLOW_DIST) followWall(rightDist, NORMAL_SPEED);
    else if (frontDist > FRONT_STOP_DIST) followWall(rightDist, SLOW_SPEED);
    else {
      if (rightDist < DESIRED_RIGHT_DIST + 2) turnLeft();
      else followWall(rightDist, SLOW_SPEED);
    }
  } else {
    stopMotors();
  }

  unsigned long now = millis();
  if (now - lastGPSTime >= 1000) {
    lastGPSTime = now;
    int co_ppm, lpg_ppm;
    readMQ2(co_ppm, lpg_ppm);
    while (Serial1.available()) gps.encode(Serial1.read());
    sendTelemetry(lpg_ppm, co_ppm, batt_voltage);
    Serial.print("Estimated battery %: ");
    Serial.println(percent * 100, 1);
  }

  int co_ppm, lpg_ppm;
  readMQ2(co_ppm, lpg_ppm);
  if (co_ppm > CO_THRESHOLD || lpg_ppm > LPG_THRESHOLD) {
    while (Serial1.available()) gps.encode(Serial1.read());
    sendTelemetry(lpg_ppm, co_ppm, batt_voltage);
    delay(500);
  }

  static unsigned long lastSave = 0;
  if (!returning && millis() - lastSave > 1000) {
    lastSave = millis();
    if (gps.location.isValid()) saveCurrentWaypoint();
  }

  delay(100);
}
