
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

// Constants
const int NORMAL_SPEED = 200;
const int SLOW_SPEED = 50;
const int DESIRED_RIGHT_DIST = 20;
const int FRONT_SLOW_DIST = 50;
const int FRONT_STOP_DIST = 20;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  steering.attach(SERVO_PIN);
  Serial.begin(9600);
}

void followWall(int rightDist, int speed) {
  if (rightDist > DESIRED_RIGHT_DIST + 2) { //check with tolerance
    steerRight();
  } else if (rightDist < DESIRED_RIGHT_DIST - 2) { // check with tolerance
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


void loop() {
  int frontDist = sonarFront.ping_cm(); //find front distance
  int rightDist = sonarRight.ping_cm(); //find right distance

  if (frontDist > FRONT_SLOW_DIST) {
    followWall(rightDist, NORMAL_SPEED);
  } else if (frontDist <= FRONT_SLOW_DIST && frontDist > FRONT_STOP_DIST) {
    followWall(rightDist, SLOW_SPEED);
  } else if (frontDist <= FRONT_STOP_DIST) {
    if (rightDist < DESIRED_RIGHT_DIST + 2) { // check with tolerance for right opening
      turnLeft();
    } else {
      followWall(rightDist, SLOW_SPEED); // If front distance is <= 20 then follow the right opening
    }
  }
  delay(100);
}


