/*
 * Μη Επανδρωμένο όχημα Ανίχνευσης Φωτιάς
 * 
 * Αυτή η έκδοση περιλαμβάνει:
 * - Παρακολούθηση ρεύματος/τάσης/ισχύος με χρήση του module INA260 για ακριβή μέτρηση
 * - Ενσωμάτωση GPS PPS pin για βελτιωμένη ακρίβεια θέσης
 * - Προηγμένη διαχείριση μπαταρίας με μη-γραμμική μοντελοποίηση LiPo
 * - Τηλεμετρία με πλήρη στοιχεία ισχύος και GPS
 * 
 * Υλικό:
 * - Arduino Mega 2560
 * - 3x Αισθητήρες υπερήχων HC-SR04 (μπροστά, 45° αριστερά, 45° δεξιά)
 * - Οδηγός κινητήρα L298N για DC κινητήρες
 * - Σερβοκινητήρας για διεύθυνση Ackermann
 * - Αισθητήρας αερίου MQ-2 για ανίχνευση φωτιάς/αερίου
 * - Μονάδα GPS NEO-6M με pin PPS
 * - Αισθητήρας INA260 Ρεύματος/Τάσης/Ισχύος
 * - Μονάδα SX1278 LoRa για τηλεμετρία
 * - Μπαταρία TATTU 2300mAh 14.8V LiPo (διάταξη 4S)
 */

#include <NewPing.h>         // Αισθητήρες υπερήχων
#include <Servo.h>           // Σερβοκινητήρας
#include <TinyGPS++.h>       // Μονάδα GPS
#include <LoRa.h>            // Μονάδα SX1278 LoRa
#include <SD.h>              // Καταγραφή σε κάρτα SD
#include <Wire.h>            // Επικοινωνία I2C
#include <INA260_WE.h>       // Αισθητήρας INA260 ρεύματος/τάσης/ισχύος
#include <MQUnifiedsensor.h> // Αισθητήρας MQ-2 για ανίχνευση αερίων

// ===== PIN DEFINITIONS =====

// --- Pins κάρτας SD ---
// Σημείωση: Η κάρτα SD χρησιμοποιεί τα hardware SPI pins στο Mega
#define SD_CS_PIN 53    // Hardware SPI CS
#define SD_MOSI 51      // Hardware SPI MOSI
#define SD_MISO 50      // Hardware SPI MISO  
#define SD_SCK 52       // Hardware SPI SCK

// Αισθητήρες υπερήχων
#define TRIG_FRONT 30 
#define ECHO_FRONT 31
#define TRIG_RIGHT 32
#define ECHO_RIGHT 33
#define TRIG_LEFT 34
#define ECHO_LEFT 35
#define MAX_DISTANCE 200 // Μέγιστη απόσταση για αισθητήρες υπερήχων (cm)

// Οδηγός κινητήρα (L298N)
#define ENA 9   // PWM αριστερού κινητήρα
#define IN1 22  
#define IN2 23  
#define ENB 10  // PWM δεξιού κινητήρα
#define IN3 24
#define IN4 25

// Servo (PWM pin)
#define SERVO_PIN 11    // PWM pin για servo

// Αισθητήρες
#define MQ2_PIN A0      // Αισθητήρας αερίου MQ-2
#define GPS_PPS_PIN 2   // Interrupt Pin PPS του GPS

// Μονάδα LoRa (SX1278)
#define LORA_SS 49      // NSS pin 
#define LORA_RST 48     // Reset pin
#define LORA_DIO0 47    // DIO0 pin

// Διευθύνσεις I2C
#define INA260_ADDRESS 0x40  // Διεύθυνση I2C του INA260

// MQ-2 Sensor Constants
#define BOARD "Arduino MEGA"
#define VOLTAGE_RESOLUTION 5
#define ADC_BIT_RESOLUTION 10
#define TYPE "MQ-2"

// ===== INITIALIZE MODULES =====

// Αισθητήρες υπερήχων
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);

// Σερβοκινητήρας διεύθυνσης
Servo steering;

// GPS
TinyGPSPlus gps;

// Αισθητήρας INA260 Ρεύματος/Τάσης/Ισχύος
INA260_WE ina260 = INA260_WE(INA260_ADDRESS);

// Αισθητήρας MQ-2 για ανίχνευση αερίων
MQUnifiedsensor MQ2(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ2_PIN, TYPE);

// ===== CONSTANTS =====

// Παράμετροι πλοήγησης
const int NORMAL_SPEED = 200;     // Κανονική ταχύτητα κινητήρα (PWM)
const int SLOW_SPEED = 50;        // Χαμηλή ταχύτητα κινητήρα (PWM)
const int DESIRED_RIGHT_DIST = 20; // Επιθυμητή απόσταση από δεξιό τοίχο (cm)
const int FRONT_SLOW_DIST = 50;   // Απόσταση για επιβράδυνση (cm)
const int FRONT_STOP_DIST = 20;   // Απόσταση για στάση (cm)

// Όρια αισθητήρων
const int CO_THRESHOLD = 50;      // Όριο CO σε ppm
const int LPG_THRESHOLD = 2100;   // Όριο LPG σε ppm

// Προδιαγραφές μπαταρίας (TATTU 2300mAh 14.8V LiPo - 4S)
const float BATTERY_CAPACITY_MAH = 2300.0;    // Χωρητικότητα μπαταρίας σε mAh
const float CELL_COUNT = 4.0;                 // Αριθμός στοιχείων σε σειρά
const float CELL_VOLTAGE_MAX = 4.2;           // Μέγιστη τάση στοιχείου (V)
const float CELL_VOLTAGE_MIN = 3.3;           // Ελάχιστη ασφαλής τάση στοιχείου (V)
const float BATTERY_VOLTAGE_MAX = CELL_COUNT * CELL_VOLTAGE_MAX; // 16.8V
const float BATTERY_VOLTAGE_MIN = CELL_COUNT * CELL_VOLTAGE_MIN; // 13.2V

// Γωνίες σερβοκινητήρα
const int SERVO_CENTER = 90;      // Θέση ευθείας
const int SERVO_LEFT = 120;       // Θέση αριστερής στροφής
const int SERVO_RIGHT = 60;       // Θέση δεξιάς στροφής

// Σταθερές χρονισμού
const unsigned long SENSOR_INTERVAL = 50;      // ms
const unsigned long TELEMETRY_INTERVAL = 1000000; // μs
const unsigned long BATTERY_INTERVAL = 100;    // ms
const unsigned long MQ2_WARMUP_TIME = 20000;   // Χρόνος προθέρμανσης MQ-2 (ms)
const unsigned long TURN_TIME_LEFT = 600;      // Χρόνος για αριστερή στροφή (ms)
const unsigned long TURN_TIME_AROUND = 1200;   // Χρόνος για αναστροφή 180° (ms)
const unsigned long TURN_TIME_RIGHT = 500;     // Χρόνος για δεξιά στροφή (ms)

// Όριο αριθμού δορυφόρων GPS
const int GPS_MIN_SATELLITES = 4;              // Ελάχιστοι δορυφόροι για καλή λήψη

// ===== VARIABLES =====

// Τήρηση Χρονισμού
unsigned long currentMillis = 0;        // Τρέχων χρόνος
unsigned long previousSensorMillis = 0; // Χρόνος τελευταίας ανάγνωσης αισθητήρων (εκτός μπαταρίας)
unsigned long previousBatteryMillis = 0; // Χρόνος τελευταίας μέτρησης αισθητήρα μπαταρίας
unsigned long startTime = 0;            // Χρόνος έναρξης αποστολής
unsigned long turnStartTime = 0;        // Χρόνος έναρξης στροφής

// Χρονισμός GPS PPS
volatile unsigned long ppsTime = 0;     // Τελευταίος παλμός PPS
volatile bool ppsReceived = false;      // Flag λήψης PPS παλμού
unsigned long lastTelemetryPPSTime = 0; // Χρόνος τελευταίας αποστολής τηλεμετρίας
unsigned long lastPPSTime = 0;          // Τελευταία ένδειξη PPS
bool ppsActive = false;                 // Flag ενεργοποίησης PPS

// Πλοήγηση
float startLat = 0;               // Αρχικό γεωγραφικό πλάτος
float startLon = 0;               // Αρχικό γεωγραφικό μήκος
bool missionStarted = false;      // Flag: Αν έχει ξεκινήσει η αποστολή
bool fireDetected = false;        // Flag: Αν έχει ανιχνευτεί φωτιά/αέριο

// Mεταβλητές αισθητήρων υπερήχων/αερίου/φωτιάς
int frontDistance = 0;            // (cm)
int rightDistance = 0;            // (cm)
int leftDistance = 0;             // (cm)
int coPPM = 0;                    // Συγκέντρωση CO (ppm)
int lpgPPM = 0;                   // Συγκέντρωση LPG (ppm)

// Μεταβλητές παρακολούθησης μπαταρίας
float batteryVoltage = 0;         // (V)
float batteryCurrent = 0;         // (mA)
float batteryPower = 0;           // (mW)
float consumedCapacity = 0;       // Καταναλωθείσα χωρητικότητα (mAh)
float chargePercentage = 100.0;      // Ποσοστό φόρτισης (%)
float remainingCapacity = BATTERY_CAPACITY_MAH; // Υπόλοιπη χωρητικότητα (mAh)
unsigned long lastBatteryTime = 0; // Τελευταίος χρόνος μέτρησης μπαταρίας
float totalEnergyConsumed = 0;    // Συνολική καταναλωθείσα ενέργεια (Wh)

// Μεταβλητές κατάστασης
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

// Σημεία καμπύλης εκφόρτισης LiPo (τάση vs SOC)
struct DischargePoint {
  float voltage;
  float soc;
};

// 4S Καμπύλη εκφόρτισης LiPo (τάση ανά στοιχείο * 4)
const DischargePoint dischargeCurve[] = {
  {16.80, 100.0}, // 4.2V x 4
  {16.50, 95.0},  // μικρή πτώση στην κορυφή
  {16.20, 90.0},  
  {15.90, 85.0},  
  {15.60, 80.0},  
  {15.30, 75.0},  
  {15.00, 70.0},  
  {14.70, 65.0},  
  {14.40, 55.0},  
  {14.10, 40.0},  
  {13.80, 25.0},  
  {13.60, 10.0},    
  {13.20, 0.0}    // 3.3V ανά στοιχείο (όριο)
};

const int DISCHARGE_CURVE_POINTS = sizeof(dischargeCurve) / sizeof(DischargePoint);

// ===== SETUP =====

void setup() {
  // Εκκίνηση σειριακής επικοινωνίας
  Serial.begin(9600);
  Serial1.begin(9600); // Για το GPS

  // Εκκίνηση I2C
  Wire.begin();
  
  // Αρχικοποίηση pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(GPS_PPS_PIN, INPUT);
  
  // Εκκίνηση σερβοκινητήρα
  steering.attach(SERVO_PIN);
  steerStraight();
  
  // Εκκίνηση αισθητήρα INA260
  if (!ina260.init()) {
    Serial.println("INA260 initialization failed!");
    while (1);
  }
  
  // Ρύθμιση INA260 για βέλτιστη απόδοση
  ina260.setMeasureMode(CONTINUOUS);
  ina260.setConversionTime(CONV_TIME_1100);
  ina260.setAverage(AVERAGE_16);
  
  Serial.println("INA260 initialized successfully");
  
  // Εκκίνηση αισθητήρα MQ-2
  MQ2.setRegressionMethod(1); // _PPM =  a*ratio^b
  MQ2.init(); 
  
  // Ρύθμιση παραμέτρων για CO (Carbon Monoxide)
  MQ2.setA(574.25); MQ2.setB(-2.222); // Παράμετροι από datasheet για CO
  
  // Βαθμονόμηση αισθητήρα (R0 calculation)
  Serial.print("Calibrating MQ-2 sensor...");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++) {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(9.83); // Καθαρός αέρας ratio, τυπικά 9.83 για MQ-2
  }
  MQ2.setR0(calcR0/10);
  Serial.println(" done!");
  
  if(isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected)");
    while(1);
  }
  if(calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground)");
    while(1);
  }
  
  Serial.println("MQ-2 sensor initialized successfully");
  
  // Εκκίνηση LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }

  // Εκκίνηση κάρτας SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    while (1);
  }
  File f = SD.open("route.csv", FILE_WRITE);
  if (f) {
    f.println("timestamp,lat,lon,voltage,current,power,soc,satellites");
    f.close();
  }
  
  // Ρύθμιση διακοπής PPS GPS
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), ppsInterrupt, RISING);
  
  // Καταγραφή χρόνου έναρξης για προθέρμανση MQ-2
  startTime = millis();
  lastBatteryTime = startTime;
  
  Serial.println("Το μΕ εκκινήθηκε. Προθέρμανση αισθητήρα MQ-2...");
  Serial.println("Ενισχυμένη διαχείριση μπαταρίας και ενεργό GPS PPS");
}

// ===== ΔΙΑΚΟΠΗ GPS PPS =====

void ppsInterrupt() {
  ppsTime = micros();
  ppsReceived = true;
}

void loop() {
  // Ενημέρωση τρέχοντος χρόνου
  currentMillis = millis();
  
  // Μηχανή καταστάσεων για αρχικοποίηση
  if (currentState == INITIALIZING) {
    // Αναμονή για προθέρμανση αισθητήρα MQ-2
    if (currentMillis - startTime >= MQ2_WARMUP_TIME) {
      // Έλεγχος για λήψη GPS
      while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
          if (gps.location.isValid() && gps.satellites.value() >= GPS_MIN_SATELLITES) {
            startLat = gps.location.lat();
            startLon = gps.location.lng();
            missionStarted = true; // Η αποστολή ξεκινάει με GPS
            saveCurrentWaypoint();
            Serial.println("Ελήφθη θέση GPS. Έναρξη αποστολής!");
            Serial.print("Θέση εκκίνησης: ");
            Serial.print(startLat, 6);
            Serial.print(", ");
            Serial.print(startLon, 6);
            break;
          }
        }
      }
      
      // Αν δεν ληφθούν δεδομένα GPS για 5 δευτερόλεπτα μετά την προθέρμανση, συνέχισε ούτως ή άλλως
      if (currentMillis - startTime >= MQ2_WARMUP_TIME + 5000 || missionStarted) {
        if (!missionStarted) {
          Serial.println("Δεν υπάρχει λήψη GPS. Έναρξη αποστολής χωρίς GPS.");
          missionStarted = true; // Η αποστολή ξεκινάει χωρίς GPS
        }
        
        // Αρχικός έλεγχος μπαταρίας
        readBatteryData();
        Serial.print("Αρχική μπαταρία: ");
        Serial.print(batteryVoltage, 2);
        Serial.print("V, ");
        Serial.print(chargePercentage, 1);
        Serial.println("% SOC");
        
        // Έτοιμο για εκκίνηση
        Serial.println("Το μΕ εκκινήθηκε και είναι έτοιμο!");
        changeState(NAVIGATING);
      }
    }
  }
  
  // Ανάγνωση δεδομένων μπαταρίας σύμφωνα με το ορισμένο χρονικό διάστημα για ακριβή μέτρηση
  if (currentMillis - previousBatteryMillis >= BATTERY_INTERVAL) {
    previousBatteryMillis = currentMillis;
    readBatteryData();
    updateBatteryState();
  }
  
  // Ανάγνωση αισθητήρων σύμφωνα με το ορισμένο χρονικό διάστημα
  if (currentMillis - previousSensorMillis >= SENSOR_INTERVAL) {
    previousSensorMillis = currentMillis;
    readSensors();
    checkForFire();
  }
  
  handleNavigation();
  
  // Αποστολή τηλεμετρίας σύμφωνα με το ορισμένο χρονικό διάστημα
  if (ppsTime - lastTelemetryPPSTime >= TELEMETRY_INTERVAL) {
    lastTelemetryPPSTime = ppsTime;
    updateGPS();
    sendTelemetry();
    printSystemStatus();
  }
}

// ===== ΣΥΝΑΡΤΗΣΕΙΣ ΔΙΑΧΕΙΡΙΣΗΣ ΜΠΑΤΑΡΙΑΣ =====

void readBatteryData() {
  batteryVoltage = ina260.getBusVoltage_V();
  batteryCurrent = ina260.getCurrent_mA();
  batteryPower = ina260.getPower_mW();
  if (batteryCurrent < 0) {
    batteryCurrent = 0;
  }
}

void updateBatteryState() {
  // Υπολογισμός διαφοράς χρόνου για μέτρηση
  unsigned long currentTime = millis();
  float deltaTimeHours = (currentTime - lastBatteryTime) / 3600000.0; // Μετατροπή ms σε ώρες
  lastBatteryTime = currentTime;
  
  // Υπολογισμός καταναλωθείσας ενέργειας σε mAh
  float consumedThisCycle = batteryCurrent * deltaTimeHours; // mAh
  consumedCapacity += consumedThisCycle;
  
  // Ενημέρωση υπόλοιπης χωρητικότητας
  remainingCapacity = BATTERY_CAPACITY_MAH - consumedCapacity;
  if (remainingCapacity < 0) remainingCapacity = 0;
  
  // Υπολογισμός SOC
  float voltageSoc = calculateVoltageBasedSOC(batteryVoltage);
  float coulombSoc = (remainingCapacity / BATTERY_CAPACITY_MAH) * 100.0;
  
  // Υπολογισμός ποσοστού φόρτισης με συνδυασμό coulomb counting και τάσης και διαφορετική βαρύτητα ανάλογα με την κατανάλωση
  if (consumedCapacity > 10) { // Μετά από κάποια κατανάλωση, εμπιστεύσου περισσότερο το coulomb counting
    chargePercentage = (coulombSoc * 0.8) + (voltageSoc * 0.2);
  } else {
    chargePercentage = (coulombSoc * 0.3) + (voltageSoc * 0.7);
  }
  chargePercentage = constrain(chargePercentage, 0, 100);
  totalEnergyConsumed += (batteryPower * deltaTimeHours) / 1000.0; // Μετατροπή mWh σε Wh
}

float calculateVoltageBasedSOC(float voltage) {
  if (voltage >= dischargeCurve[0].voltage) {
    return 100.0;
  }
  if (voltage <= dischargeCurve[DISCHARGE_CURVE_POINTS - 1].voltage) {
    return 0.0;
  }
  
  // Βρες σε ποιο εύρος τάσης του πίνακα είναι η μέτρηση
  for (int i = 0; i < DISCHARGE_CURVE_POINTS - 1; i++) {
    if (voltage <= dischargeCurve[i].voltage && voltage >= dischargeCurve[i + 1].voltage) {
      // Βρες την ακριβή διαφορά τάσης μεταξύ των δύο γειτονικών τιμών του πίνακα
      float voltageRange = dischargeCurve[i].voltage - dischargeCurve[i + 1].voltage;
      // Βρες την ακριβή διαφορά SOC μεταξύ των δύο γειτονικών τιμών του πίνακα
      float socRange = dischargeCurve[i].soc - dischargeCurve[i + 1].soc;
      // Υπολόγισε την διαφορά της μέτρησης από την αμέσως μεγαλύτερη τιμή τάσης του πίνακα
      float voltageOffset = dischargeCurve[i].voltage - voltage;
      // Επέστρεψε το υπολογιζόμενο ποσοστό χωρητικότητας που απομένει
      return dischargeCurve[i].soc - (voltageOffset / voltageRange) * socRange;
    }
  }
  
  return 50.0; // Επιστροφή προεπιλογής
}

float estimateRemainingTime() {
  // Εκτίμηση υπολειπόμενου χρόνου βάσει τρέχουσας κατανάλωσης
  if (batteryCurrent <= 0) {
    return 999.0; // Άπειρος χρόνος αν δεν υπάρχει κατανάλωση
  }
  
  // Υπολογισμός υπολειπόμενου χρόνου σε ώρες
  float remainingTimeHours = remainingCapacity / batteryCurrent;
  
  // Μετατροπή σε λεπτά
  return remainingTimeHours * 60.0;
}

// ===== ΔΙΑΧΕΙΡΙΣΗ ΚΑΤΑΣΤΑΣΕΩΝ =====

void changeState(RobotState newState) {
  if (currentState != newState) {
    currentState = newState;
    stateChanged = true;
    
    // Καταγραφή χρόνου έναρξης για καταστάσεις με χρονισμό
    if (newState == TURNING_LEFT || newState == TURNING_RIGHT || newState == TURNING_AROUND) {
      turnStartTime = currentMillis;
    }
    
    // Έξοδος αποσφαλμάτωσης
    Serial.print("Η κατάσταση άλλαξε σε: ");
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

// ===== ΣΥΝΑΡΤΗΣΕΙΣ ΠΛΟΗΓΗΣΗΣ =====

void handleNavigation() {
  switch (currentState) {
    case NAVIGATING:
      // Η κίνηση βασίζεται στις μετρήσεις των αισθητήρων υπερήχων
      if (frontDistance > FRONT_SLOW_DIST) {
        // Αν δεν υπάρχει εμπόδιο μπροστά, προχωρά κανονικά κρατώντας την επιθυμητή απόσταση από τον δεξί τοίχο
        followWall(rightDistance, NORMAL_SPEED);
      } 
      else if (frontDistance <= FRONT_SLOW_DIST && frontDistance > FRONT_STOP_DIST) {
        // Εμπόδιο μπροστά, αλλά όχι πολύ κοντά
        followWall(rightDistance, SLOW_SPEED);
      } 
      else if (frontDistance <= FRONT_STOP_DIST) {
        // Εμπόδιο πολύ κοντά μπροστά, σταμάτα
        stopMotors();
        
        // Έλεγχος αν δεξιά ή αριστερά υπάρχουν εμπόδια
        if (rightDistance > DESIRED_RIGHT_DIST + 5) {
          // Ελέυθερα από δεξιά
          changeState(TURNING_RIGHT);
        } 
        else if (leftDistance > DESIRED_RIGHT_DIST + 5) {
          // Ελεύθερα από αριστερά
          changeState(TURNING_LEFT);
        } 
        else {
          // Δεξιά και αριστερά υπάρχουν εμπόδια, κάνε αναστροφή
          changeState(TURNING_AROUND);
        }
      }
      break;
      
    case TURNING_LEFT:
      // Εκτέλεση αριστερής στροφής με υποβοήθηση αριστερής περιστροφής
      if (stateChanged) {
        // Στρίβει τις εμπρός ρόδες πρώτα αριστερά 
        steerLeft();
        // Κάνει αριστερή περιστροφή με τις πίσω ρόδες για να βοηθήσει την κίνηση
        analogWrite(ENA, 100); // Ο αριστερός τροχός περιστρέφεται με μικροτερη ταχύτητα λόγω μικρού τόξου
        analogWrite(ENB, 180); // Ο δεξιός τροχός περιστρέφεται με μεγαλύτερη ταχύτητα λόγω μεγάλου τόξου
        // Ο αριστερός τροχός περιστέφεται αντίθετα προς την κίνηση
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        // Ο δεξιός τροχός περιστέφεται προς την κίνηση
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        stateChanged = false;
      }
      
      // Έλεγχος αν η αριστερή κίνηση έχει ολοκληρωθεί
      if (currentMillis - turnStartTime >= TURN_TIME_LEFT) {
        stopMotors();
        steerStraight();
        changeState(NAVIGATING);
      }
      break;
      
    case TURNING_RIGHT:
      // Εκτέλεση δεξιάς στροφής με υποβοήθηση δεξιάς περιστροφής
      if (stateChanged) {
        // Στρίβει τις εμπρός ρόδες πρώτα δεξιά 
        steerRight();
        // Κάνει δεξιά περιστροφή με τις πίσω ρόδες για να βοηθήσει την κίνηση
        analogWrite(ENA, 180); // Ο αριστερός τροχός περιστρέφεται με μεγαλύτερη ταχύτητα λόγω μεγάλου τόξου
        analogWrite(ENB, 100); // Ο δεξιός τροχός περιστρέφεται με μικροτερη ταχύτητα λόγω μικρού τόξου
        // Ο αριστερός τροχός περιστέφεται προς την κίνηση
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        // Ο δεξιός τροχός περιστέφεται αντίθετα προς την κίνηση
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        stateChanged = false;
      }
      
      // Έλεγχος αν η δεξιά κίνηση έχει ολοκληρωθεί
      if (currentMillis - turnStartTime >= TURN_TIME_RIGHT) {
        steerStraight();
        changeState(NAVIGATING);
      }
      break;
      
    case TURNING_AROUND:
      // Εκτέλεση αριστερής αναστροφής
      if (stateChanged) {
        steerLeft();
        // Κάνει αριστερή περιστροφή με τις πίσω ρόδες με την ίδια ταχύτητα αλλά αντίθετη φορά
        analogWrite(ENA, 200);
        analogWrite(ENB, 200);
        // Ο αριστερός τροχός περιστέφεται αντίθετα προς την κίνηση
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        // Ο δεξιός τροχός περιστέφεται προς την κίνηση
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        stateChanged = false;
      }
      
      // Έλεγχος αν η αναστροφή έχει ολοκληρωθεί
      if (currentMillis - turnStartTime >= TURN_TIME_AROUND) {
        stopMotors();
        steerStraight();
        changeState(NAVIGATING);
      }
      break;
      
    case STOPPED:
      // Το robot έχει σταματήσει να κινείται
      stopMotors();
      break;
  }
}

void followWall(int rightDist, int speed) {
  // Κανόνας δεξιού χεριού
  if (rightDist > DESIRED_RIGHT_DIST + 5) {
    // Αν έχεις απομακρυνθεί από τον τοίχο, πλησίασε
    steerRight();
  } 
  else if (rightDist < DESIRED_RIGHT_DIST - 2) {
    // Αν είσαι πολύ κοντά στον τοίχο, απομακρύνσου
    steerLeft();
  } 
  else {
    // ΕΠιθυμητή απόσταση από τον τοίχο, κράτα ευθεία
    steerStraight();
  }
  
  // Προχώρησε εμπρός με την καθορισμένη ταχύτητα
  moveForward(speed);
}

// ===== Λειτουργίες Αισθητήρων =====

void readSensors() {
  // Ανάγνωση αισθητήρων υπερήχων
  frontDistance = sonarFront.ping_cm();
  rightDistance = sonarRight.ping_cm();
  leftDistance = sonarLeft.ping_cm();
  
  // Αν η απόσταση είναι 0 (εκτός εμβέλειας), θέσε στη μέγιστη τιμή
  if (frontDistance == 0) frontDistance = MAX_DISTANCE;
  if (rightDistance == 0) rightDistance = MAX_DISTANCE;
  if (leftDistance == 0) leftDistance = MAX_DISTANCE;
  
  // Ανάγνωση αισθητήρα MQ-2
  readMQ2(coPPM, lpgPPM);
}

void readMQ2(int &co_ppm, int &lpg_ppm) {
  // Ενημέρωση δεδομένων αισθητήρα MQ-2
  MQ2.update(); // Ανάγνωση τάσης από το analog pin
  
  // Ανάγνωση CO (Carbon Monoxide) με παραμέτρους για CO
  MQ2.setA(574.25); MQ2.setB(-2.222); // Παράμετροι για CO από datasheet
  co_ppm = (int)MQ2.readSensor(); // Ανάγνωση CO σε ppm
  
  // Ανάγνωση LPG με παραμέτρους για LPG
  MQ2.setA(44771); MQ2.setB(-3.245); // Παράμετροι για LPG από datasheet
  lpg_ppm = (int)MQ2.readSensor(); // Ανάγνωση LPG σε ppm
  
  // Περιορισμός τιμών σε λογικά όρια
  co_ppm = constrain(co_ppm, 0, 10000);
  lpg_ppm = constrain(lpg_ppm, 0, 10000);
}

void updateGPS() {
  // Ανάγνωση δεδομένων GPS
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.location.isValid() && gps.satellites.value() >= GPS_MIN_SATELLITES) {

        saveCurrentWaypoint();
      }
    }
  }
  
  // Διαχείριση συγχρονισμού και επαλήθευσης PPS
  if (ppsReceived) {
    ppsReceived = false;
    
    // Επαλήθευση ακρίβειας χρονισμού PPS
    if (lastPPSTime > 0) {
      unsigned long ppsInterval = ppsTime - lastPPSTime;
      // Το PPS πρέπει να είναι ακριβώς 1 δευτερόλεπτο (1.000.000 μικροδευτερόλεπτα)
      if (abs((long)(ppsInterval - 1000000)) < 1000) {  // Εντός ανοχής 1ms
        ppsActive = true;
        Serial.println("PPS: Εντοπίστηκε ακριβές σήμα 1Hz");
      } else {
        Serial.print("PPS: Σφάλμα χρονισμού - διάστημα: ");
        Serial.print(ppsInterval);
        Serial.println(" μs (αναμενόμενο: 1.000.000 μs)");
      }
    }
    lastPPSTime = ppsTime;
    
    // Χρήση PPS για ακριβή συγχρονισμό δεδομένων GPS
    if (gps.location.isValid() && ppsActive) {
      // Τα δεδομένα GPS είναι πλέον συγχρονισμένα σε ακρίβεια μικροδευτερολέπτου
      Serial.print("PPS-συγχρονισμένο GPS στο: ");
      Serial.print(ppsTime);
      Serial.println(" μs");
    }
  }
  
  // Έλεγχος για timeout PPS (καμία λήψη για >2 δευτερόλεπτα)
  if (ppsActive && (micros() - ppsTime) > 2000000) {
    ppsActive = false;
    Serial.println("ΠΡΟΕΙΔΟΠΟΙΗΣΗ: Απώλεια σήματος PPS");
  }
}

// ===== ΣΥΝΑΡΤΗΣΕΙΣ ΕΛΕΓΧΟΥ ΚΙΝΗΤΗΡΩΝ =====

void moveForward(int speed) {
  // Εμπρός κίνηση με καθορισμένη ταχύτητα
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  // Σταμάτάει η κίνηση των dc motor
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void steerStraight() {
  // Οι εμπρός ρόδες είναι ευθεία
  steering.write(SERVO_CENTER);
}

void steerLeft() {
  // Οι εμπρός ρόδες είναι αριστερά
  steering.write(SERVO_LEFT);
}

void steerRight() {
  // Οι εμπρός ρόδες είναι δεξιά
  steering.write(SERVO_RIGHT);
}

// ===== ΑΝΙΧΝΕΥΣΗ ΦΩΤΙΑΣ ΚΑΙ ΤΗΛΕΜΕΤΡΙΑ =====

void checkForFire() {
  // Έλεγχος αν έχει ανιχνευτεί φωτιά ή αέριο
  if (coPPM > CO_THRESHOLD || lpgPPM > LPG_THRESHOLD) {
    
    if (!fireDetected) {
      // Αποστολή ΕΝΟΣ επείγοντος μηνύματος τηλεμετρίας όταν ανιχνευτεί φωτιά / αερίο μέχρι να πέσουν οι τιμές 
      fireDetected = true;
      Serial.println("ALERT: Fire/Gas detected!");
      
      sendEmergencyTelemetry();
    }
  } else {
    // Επαναφορά flag ανιχνευσης φωτιάς / αερίου όταν πέσουν οι τιμές κάτω από τα καθορισμένα όρια
    fireDetected = false;
  }
}

void sendTelemetry() {
  // Μορφή τηλεμετρίας: lat, lon, LPG_ppm, CO_ppm, batt_voltage, 
  // soc_percent, remaining_minutes
  String telemetryData = formatTelemetryData();
  
  // Αποστολή μέσω LoRa
  LoRa.beginPacket();
  LoRa.print("Telemetry: ");
  LoRa.print(telemetryData);
  LoRa.endPacket();
  
  // Έξοδος αποσφαλμάτωσης
  Serial.print("Απεστάλη τηλεμετρία: ");
  Serial.println(telemetryData);
}

void sendEmergencyTelemetry() {
  // Αποστολή επείγουσας τηλεμετρίας με τρέχουσα θέση και μετρήσεις αερίου
  String emergencyData = formatTelemetryData();
  
  // Αποστολή μέσω LoRa με υψηλότερη προτεραιότητα
  LoRa.beginPacket();
  LoRa.print("EMERGENCY: ");
  LoRa.print(emergencyData);
  LoRa.endPacket();
  
  // Έξοδος αποσφαλμάτωσης
  Serial.print("ΕΠΕΙΓΟΥΣΑ ΤΗΛΕΜΕΤΡΙΑ: ");
  Serial.println(emergencyData);
}

String formatTelemetryData() {
  // Μορφή δεδομένων τηλεμετρίας
  String data = "";
  
  if (gps.location.isValid() && gps.satellites.value() >= GPS_MIN_SATELLITES) {
    data += String(gps.location.lat(), 6) + ",";
    data += String(gps.location.lng(), 6) + ",";
  } else {
    data += "0.000000,0.000000,";
  }
  
  data += String(lpgPPM) + ",";
  data += String(coPPM) + ",";
  data += String(batteryVoltage, 2) + ",";
  data += String(chargePercentage, 1) + ",";
  data += String(estimateRemainingTime(), 1);
  
  return data;
}


void saveCurrentWaypoint() {
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  
  File f = SD.open("route.csv", FILE_WRITE);
  if (f) {
    f.print(lat, 6); f.print(',');
    f.print(lon, 6); f.print(',');
    f.close();
  }
}

// ===== ΚΑΤΑΣΤΑΣΗ ΚΑΙ ΠΑΡΑΚΟΛΟΥΘΗΣΗ =====

void printSystemStatus() {
  // Εκτύπωση πλήρους κατάστασης συστήματος
  unsigned long missionTime = (currentMillis - startTime) / 1000; // δευτερόλεπτα
  
  Serial.println("=== ΚΑΤΑΣΤΑΣΗ ΣΥΣΤΗΜΑΤΟΣ ===");
  Serial.print("Χρόνος αποστολής: ");
  Serial.print(missionTime / 60); // λεπτά
  Serial.print(":");
  Serial.print(missionTime % 60); // δευτερόλεπτα
  Serial.println();
  
  // Κατάσταση μπαταρίας
  Serial.print("Μπαταρία: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V, ");
  Serial.print(batteryCurrent, 1);
  Serial.print("mA, ");
  Serial.print(batteryPower, 1);
  Serial.print("mW, SOC: ");
  Serial.print(chargePercentage, 1);
  Serial.print("%, Υπόλοιπο: ");
  Serial.print(estimateRemainingTime(), 0);
  Serial.println(" min");
  
  // Κατάσταση GPS
  if (gps.location.isValid()) {
    Serial.print("GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.println("GPS: Χωρίς λήψη");
  }
  
  // Κατάσταση αισθητήρων
  Serial.print("Αποστάσεις - Μπροστά: ");
  Serial.print(frontDistance);
  Serial.print("cm, Δεξιά: ");
  Serial.print(rightDistance);
  Serial.print("cm, Αριστερά: ");
  Serial.print(leftDistance);
  Serial.println("cm");
  
  Serial.print("Αέρια - CO: ");
  Serial.print(coPPM);
  Serial.print("ppm, LPG: ");
  Serial.print(lpgPPM);
  Serial.println("ppm");
  
  // Κατανάλωση ενέργειας
  Serial.print("Συνολική καταναλωθείσα ενέργεια: ");
  Serial.print(totalEnergyConsumed, 2);
  Serial.println(" Wh");
  
  Serial.println("=====================");
}
