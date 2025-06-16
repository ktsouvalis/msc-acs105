#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Servo.h>

// Ultrasonic sensor pins
#define TRIG_F 2
#define ECHO_F 3
#define TRIG_L 4
#define ECHO_L 5
#define TRIG_R 6
#define ECHO_R 7

// MQ-2 sensor
#define MQ2_PIN A0

// Servo
#define SERVO_PIN 8

// Motor pins
#define ENA 9
#define IN1 10
#define IN2 11
#define ENB 12
#define IN3 13
#define IN4 A1

// Battery voltage sense pin
#define BATT_PIN A2

TinyGPSPlus gps;
Servo steering;

unsigned long lastGPSTime = 0;

// Thresholds
const int CO_THRESHOLD = 50;    // ppm
const int LPG_THRESHOLD = 2100; // ppm

// Calibration values for battery voltage divider
const float BATT_R1 = 10000.0; // 10k
const float BATT_R2 = 2000.0;  // 2k
const float ADC_REF = 5.0;     // V
const float ADC_RES = 1023.0;

enum State {
  GO_FORWARD,
  TURN_RIGHT,
  TURN_LEFT,
  STOPPED
};

State currentState = GO_FORWARD;
State previousState = GO_FORWARD;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // GPS
  Serial2.begin(9600); // LoRa
  LoRa.begin(433E6);
  steering.attach(SERVO_PIN);

  // Ultrasonic sensors
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  // Motors
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // MQ-2
  pinMode(MQ2_PIN, INPUT);
  pinMode(BATT_PIN, INPUT);
}

long measureSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // timeout 25ms
  long distance = duration * 0.034 / 2;
  return distance;
}

void setMotors(int pwmA, int pwmB) {
  // Motor A (left)
  if (pwmA > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmA);
  } else if (pwmA < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwmA);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  // Motor B (right)
  if (pwmB > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmB);
  } else if (pwmB < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwmB);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// Example: simulate LPG reading from MQ2
// NOTE: This is a simplified approach. The mapping below does NOT reflect real gas concentrations.
// Proper calibration with known gas concentrations and the MQ-2 datasheet is required for accurate ppm values.
void readMQ2(int &co_ppm, int &lpg_ppm) {
  int mq2_raw = analogRead(MQ2_PIN);
  co_ppm = map(mq2_raw, 0, 1023, 0, 1000);   // simplified approach: not accurate, just for demonstration
  lpg_ppm = map(mq2_raw, 0, 1023, 0, 5000);  // simplified approach: not accurate, just for demonstration
}

float readBatteryVoltage() {
  int raw = analogRead(BATT_PIN);
  float vout = (raw * ADC_REF) / ADC_RES;
  float vin = vout / (BATT_R2 / (BATT_R1 + BATT_R2));
  return vin;
}

// Telemetry format: lat,lon,LPG_ppm,CO_ppm,batt_voltage
void sendTelemetry(int lpg_ppm, int co_ppm, float batt_voltage) {
  String msg = String(gps.location.lat(), 6) + "," +
               String(gps.location.lng(), 6) + "," +
               String(lpg_ppm) + "," +
               String(co_ppm) + "," +
               String(batt_voltage, 2);
  LoRa.println(msg);
}

// Helper: Convert RPM to PWM value (assuming 125 RPM = 255 PWM)
int rpmToPwm(float rpm) {
  return constrain((int)(rpm / 125.0 * 255), 0, 255);
}

void loop() {
  // GPS parsing
  while (Serial1.available()) gps.encode(Serial1.read());
  unsigned long now = millis();
  static unsigned long lastSample = 0;
  if (now - lastGPSTime >= 1000) {
    lastGPSTime = now;
    int co_ppm, lpg_ppm;
    readMQ2(co_ppm, lpg_ppm);
    float batt_voltage = readBatteryVoltage();
    sendTelemetry(lpg_ppm, co_ppm, batt_voltage);
  }

  // Read distances from ultrasonic sensors
  int dF = measureSensor(TRIG_F, ECHO_F);
  int dR = measureSensor(TRIG_R, ECHO_R);
  int dL = measureSensor(TRIG_L, ECHO_L);

  // Right-hand rule logic with speed control and obstacle avoidance
  int speed = 0;
  int direction = 90;

  // === Sensor decision logic ===
  if (dR > 20) {
    // Right is clear, try to turn right
    if (dF < 50) {
      currentState = (dF > 20) ? TURN_RIGHT : STOPPED;
    } else {
      currentState = TURN_RIGHT;
    }
  } else {
    // Right is blocked
    if (dF <= 20) {
      currentState = TURN_LEFT;
    } else if (dF <= 50) {
      currentState = GO_FORWARD; // but slow
    } else {
      currentState = GO_FORWARD; // fast
    }
  }

  // === Action based on state ===
  if (currentState != previousState) {
    switch (currentState) {
      case TURN_RIGHT:
        speed = 120;
        direction = 45;
        break;

      case TURN_LEFT:
        speed = 100;
        direction = 135;
        break;

      case GO_FORWARD:
        speed = (dF <= 50) ? 60 : 180;
        direction = 90;
        break;

      case STOPPED:
        speed = 0;
        direction = 90;
        break;
    }

    setMotors(speed, speed);
    steering.write(direction);

    previousState = currentState;
  }

  // MQ-2 logic: detect CO/LPG and send telemetry if threshold exceeded
  int co_ppm, lpg_ppm;
  readMQ2(co_ppm, lpg_ppm);
  float batt_voltage = readBatteryVoltage();
  if (co_ppm > CO_THRESHOLD || lpg_ppm > LPG_THRESHOLD) {
    Serial.print("CO: "); Serial.print(co_ppm);
    Serial.print(" ppm, LPG: "); Serial.print(lpg_ppm);
    Serial.print(" ppm, Battery: "); Serial.println(batt_voltage, 2);
    sendTelemetry(lpg_ppm, co_ppm, batt_voltage);
    delay(500); // avoid spamming
  }
}
