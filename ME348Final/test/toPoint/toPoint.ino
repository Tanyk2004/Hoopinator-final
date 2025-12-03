#include <QTRSensors.h>
#include <AStar32U4Motors.h>

// --- HARDWARE CONFIGURATION ---
// Ultrasonic Pins
const int trigPinFront = 7;
const int echoPinFront = 6;
const int trigPinRight = 11; // CHANGED FROM 11 TO AVOID CONFLICT
const int echoPinRight = 8;

// QTR Sensors
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Motor Driver
AStar32U4Motors m;

// Variables
uint16_t linePosition = 3500;
int isCross = 0;
int leftMotorCmd = 0;
int rightMotorCmd = 0;

// Serial Data Handling
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
boolean newData = false;

// Speed of Sound for Ultrasonic
#define SPEED_SOUND 0.0343 // cm/us

void setup() {
  Serial.begin(115200);
  
  // Ultrasonic Setup
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  // QTR Setup
  qtr.setTypeRC();
  // Note: Ensure Pin 11 is NOT used here if used for Ultrasonic
  qtr.setSensorPins((const uint8_t[]){18,20,21,22,23,16,15,14}, SensorCount); 
  qtr.setEmitterPin(4);
  
  // Calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("<Arduino Ready>");
}

void loop() {
  // 1. READ LINE SENSORS (Your specific repair logic included)
  linePosition = qtr.readLineBlack(sensorValues);
  
  // Filter noise < 300
  for (int i=0; i<8; i++){
     if (sensorValues[i] < 300) sensorValues[i] = 0;
  }
  
  // Repair: All zero
  bool allZero = true;
  for(int i=0; i<8; i++) { if(sensorValues[i] > 0) allZero = false; }
  if (allZero) linePosition = 9500; // Using 9500 as "No Line" code

  // Repair: Edge cases
  if (sensorValues[0] > 0 && sensorValues[1]==0 && sensorValues[2]==0) linePosition = 1000;
  if (sensorValues[7] > 0 && sensorValues[6]==0 && sensorValues[5]==0) linePosition = 5000;
  
  // Clamp
  if (linePosition != 9500) {
    if (linePosition > 5000) linePosition = 5000;
    if (linePosition < 1000) linePosition = 1000;
  }

  // Cross Detection (Sensors 0 and 7 active)
  if ((sensorValues[7] > 500) && (sensorValues[0] > 500)) {
    isCross = 1;
  } else {
    isCross = 0;
  }

  // 2. READ ULTRASONICS
  float distFront = readUltrasonic(trigPinFront, echoPinFront);
  float distRight = readUltrasonic(trigPinRight, echoPinRight);

  // 3. SEND DATA TO PI (Format: <linePos,isCross,distFront,distRight>)
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 40) { // ~25Hz
    Serial.print("<");
    Serial.print(linePosition);
    Serial.print(",");
    Serial.print(isCross);
    Serial.print(",");
    Serial.print(distFront);
    Serial.print(",");
    Serial.print(distRight);
    Serial.println(">");
    lastSend = millis();
  }

  // 4. RECEIVE MOTOR COMMANDS
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChar, receivedChars);
    parseData();
    newData = false;
    
    m.setM1Speed(leftMotorCmd);
    m.setM2Speed(rightMotorCmd);
  }
}

float readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float duration = pulseIn(echo, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999; // No echo
  return (duration * SPEED_SOUND) / 2;
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) ndx = numChars - 1;
            }
            else {
                receivedChars[ndx] = '\0'; // terminate string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {
    char *strtokIndexer;
    strtokIndexer = strtok(tempChar, ",");
    if (strtokIndexer != NULL) leftMotorCmd = atoi(strtokIndexer);
    
    strtokIndexer = strtok(NULL, ",");
    if (strtokIndexer != NULL) rightMotorCmd = atoi(strtokIndexer);
}
