#include <AStar32U4Motors.h>
#include <float.h>
#include <limits.h>
#include <QTRSensors.h>

enum RobotStates {
  INIT = 0,
  DEAD_RECKONING = 1,
  LINE_FOLLOWING = 2,
  AIMING = 3,
  SHOOTING = 4
};

enum Directions {
  FORWARD = 1,
  BACKWARD = -1,
  LEFT = 2,
  RIGHT = -2
};

#define MOTOR_SPEED 100

uint16_t LINE_POSITION_CENTER = 3500;
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition = LINE_POSITION_CENTER;

const int trigPinFront = 7;
const int echoPinFront = 6;

const int trigPinRight = 11;
const int echoPinRight = 8;

#define Kp 0
#define Ki 0
#define Kd 0

#define SPEED_SOUND 0.0343  // cm/microSeconds
#define NUM_POINTS 200

#define START_BOX_X 45.72
#define START_BOX_Y 76.2

#define FIELD_X 182.88
#define FIELD_Y 182.88

AStar32U4Motors m;

int leftMotorVelocity = 0;
int rightMotorVelocity = 0;

int leftMotor = 0;
int rightMotor = 0;

RobotStates currentState = INIT;

unsigned long currentTime;
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];  // temporary array used for parsing
boolean newData = false;
float distanceFront;
float distanceRight;
float flt_infinity = 90000;
float minDistanceSeen = flt_infinity;       // initialize to inf
float lastSeenWallDistance = flt_infinity;  // initialize to inf
Directions dr_lastDirection = LEFT;
Directions wallBeingFaced = FORWARD;
unsigned long lastMinChanged = 0;

float getUltrasonicReading(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * SPEED_SOUND) / 2;

  return distance;
}

void publishSensorData() {
  // Placeholder for publishing sensor data if needed
  Serial.print(currentState);
  Serial.print(",");
  Serial.print(distanceFront);
  Serial.print(",");
  Serial.print(distanceRight);
  Serial.print(",");
  Serial.print(linePosition);
  Serial.print(",");
  Serial.print(leftMotor);
  Serial.print(",");
  Serial.println(rightMotor);

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
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminates the string, frankly unsure why I need this
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

void stopMotors() {
  leftMotor = 0;
  rightMotor = 0;
  commandMotors();
}

void dr_turn(Directions dir) {
  if (dir == LEFT) {
    dr_turnLeft();
  } else if (dir == RIGHT) {
    dr_turnRight();
  }
}

void dr_turnLeft() {  // make it turn 90 degrees
  
  leftMotor = -1 * (MOTOR_SPEED -10);
  rightMotor = MOTOR_SPEED -10;
  commandMotors();
  delay(50);
}

void dr_turnRight() {  // make it turn 90 degrees
  
  leftMotor = MOTOR_SPEED -10;
  rightMotor = -1 * (MOTOR_SPEED - 10);
  commandMotors();
  delay(50);
}

void dr_turnAround(Directions bias) {
  leftMotor = MOTOR_SPEED / 2;
  rightMotor = -MOTOR_SPEED / 2;
  commandMotors();
  delay(800);  // adjust this delay to achieve a 180-degree turn
  stopMotors();
  // TODO: Add the bias
  if (bias == LEFT) {
    leftMotor = -MOTOR_SPEED;
    rightMotor = MOTOR_SPEED;
    delay(100);
  } else {
    dr_lastDirection = RIGHT;
    leftMotor = MOTOR_SPEED;
    rightMotor = -MOTOR_SPEED;
    delay(100);
  }
}

void parseData() {

  char *strtokIndexer;  //doing char * allows strtok to increment across my string properly frankly im not sure why... something to do with pointers that I dont expect students to understand

  strtokIndexer = strtok(tempChar, ",");  //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  leftMotor = atoi(strtokIndexer);        //converts strtokIndexer into a int
  strtokIndexer = strtok(NULL, ",");      //setting the first input to null causes strtok to continue looking for commas in tempChar starting from where it left off, im not really sure why
  rightMotor = atoi(strtokIndexer);
  strtokIndexer = strtok(NULL, ",");
  currentState = static_cast<RobotStates>(atoi(strtokIndexer));
}

bool faceWallPerpendicularly() {

  //implement a simple proportional controller to face the wall perpendicularly using the front and right ultrasonic sensors
  lastSeenWallDistance = flt_infinity;
  Serial.print("Last Seen wall distance: ");
  Serial.println(lastSeenWallDistance);

  bool facingWall = false;

  // debug
  Serial.println("Turning toward wall!");
  delay(3000);

  bool lastTurnOpposite = false;

  while (!facingWall) {
    // publishSensorData();
    // facingWall = faceWallPerpendicularly();
    distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
    distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
    unsigned long currentMillis = millis();
    Serial.print("Last Direction: ");
    Serial.println(dr_lastDirection);


    // find the local minimum distance to the wall
    if (distanceFront <= lastSeenWallDistance) {
      Serial.println("Distance decreasing");
      lastSeenWallDistance = distanceFront;
      if (distanceFront < minDistanceSeen) {
        lastMinChanged = millis();
        minDistanceSeen = distanceFront;
      }

      dr_turn(dr_lastDirection);
      lastTurnOpposite = false;

      delay(1000);
    } else {
      // if the distance starts increasing, turn the other way
      Serial.println("Distance increasing");
      if (currentMillis > lastMinChanged + 5000) {
        // if it's been a while since the last minimum, assume we're done
        Serial.println("Found the closest wall!");
        stopMotors();
        delay(1000);
        return;
      }
      if (!lastTurnOpposite) { // we did not change direction last time
        lastTurnOpposite = true;
        dr_lastDirection = -1 * dr_lastDirection;
      } else { // we changed direction last time so we should not this time
        lastTurnOpposite = false;
      }
      dr_turn(dr_lastDirection);


    }
    delay(50);
  }
}

void turnTowardRightWall() {
  distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
  Serial.println("Turning right first leg");
  while (distanceFront > lastSeenWallDistance) {  // turn right until distance starts decreasing
    publishSensorData();
    lastSeenWallDistance = distanceFront;
    dr_turnRight();
    delay(50);
    distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  }
  Serial.println("Turning right second leg");
  delay(3000);
  while (distanceFront < lastSeenWallDistance) {  // turn right until distance starts increasing
    publishSensorData();
    lastSeenWallDistance = distanceFront;
    dr_turnRight();
    delay(50);
    distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  }
  stopMotors();
}

void turnTowardLeftWall() {
  distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
  Serial.println("Turning right first leg");
  while (distanceFront > lastSeenWallDistance) {  // turn right until distance starts decreasing
    publishSensorData();
    lastSeenWallDistance = distanceFront;
    dr_turnLeft();
    delay(50);
    distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  }
  Serial.println("Turning right second leg");
  delay(3000);
  while (distanceFront < lastSeenWallDistance) {  // turn right until distance starts increasing
    publishSensorData();
    lastSeenWallDistance = distanceFront;
    dr_turnLeft();
    delay(50);
    distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  }
  stopMotors();
}

void calibrateSensors(){

  //THE SENSORS ONLY CALIBRATE WHEN YOU UPLOAD NEW ARDUINO CODE TO THE ASTAR. after that the sensors STAY calibrated as long as the Astar has power.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
                                   ///while calibrating, move the sensor over the line a couple times

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10s seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
}



int repairLineSensors() {
  for (int i=0; i<= 7; i++){
   if (sensorValues[i] <300){
       sensorValues[i]=0;     
    }
  }
  if (sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
  linePosition=0;
  }

  if (sensorValues[0] >0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
    linePosition=1000;
  } 
  if (sensorValues[7] >0 && sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0){
    linePosition=5000;
  }

  //REPAIR NUMBER 4: there are still situations where linePosition is somehow greater than 5000 or 0<linePosition<1000, so I am hard capping linePosition to be between 1000 and 5000 when linePosition is greater
  //than zero.
  if (linePosition > 5000){
    linePosition = 5000;
  }
  if (linePosition < 1000 && linePosition > 0){
    linePosition = 1000;
  }

  // No line is detected
  if (sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
    linePosition = 9500;
  }

  //this loop uses the leftmost and rightmost sensors to determin if the robot is at a cross. If both of those sensors read high, then the robot is at a cross. 
  if ((sensorValues[7] > 500) && (sensorValues[0] > 500)){
    return 1;
  }
  else {
    return 0;
  }
}

void handleLineFollowing() {

  int crossesSeen = 0;
  float LineKp = 0.1;
  float LineKI = 0;
  float LineKD = 0.2;

  unsigned long previousMillis = millis();
  float prevError = -5000;
  float error = 0;
  float totalError = 0;
  // float output = 0;
  int numIterations = 0;
  int iterationLastReset = 0;

  bool debouncing = false;
  bool first = true;

  while (crossesSeen < 3) {
    linePosition = qtr.readLineBlack(sensorValues);
    int res = repairLineSensors();
    if (res == 1 && !debouncing) {
      debouncing = true;
    } else if (res == 0 && debouncing) {
      debouncing = false;
      crossesSeen++;

      leftMotor = 0;
      rightMotor = 0;
      commandMotors();
      delay(5000);
    } 
    // if none of the sensors see a line drive forward
    if (linePosition == 9500) {
      leftMotor = MOTOR_SPEED;
      rightMotor = MOTOR_SPEED;
      commandMotors();
      continue;
    }

    // Implement a simple PID controller for line following
    

    error = (float)linePosition - 3500.0f;  // assuming 3500 is the
    if (first) {
      prevError = error;
      first = false;
    }

    unsigned long currentMillis = millis();
    int deltaTime = currentMillis - previousMillis;
    if (deltaTime == 0) {
      deltaTime = 1;  // prevent division by zero
    }
    totalError += error * deltaTime;

    if (numIterations - iterationLastReset >= 100) {
      totalError = 0;
      iterationLastReset = numIterations;
    }

    float derivative = (error - prevError) / deltaTime;   // placeholder for derivative term
    // output += LinedsaKp * error; //+ LineKD * derivative + LineKI * totalError;  // + LineKI * integral + LineKD * derivative;
    float output =  LineKp * error + LineKD * derivative + LineKI * totalError;
    output = constrain(output, -40.0, 40.0);

    Serial.print("output: ");
    Serial.println(output);
    Serial.print("linePosition: ");
    Serial.println(linePosition);
    Serial.print("error: ");
    Serial.println(error);
    Serial.print("prevError: ");
    Serial.println(prevError);
    Serial.print("crossesSeen: ");
    Serial.println(crossesSeen);
    Serial.println("Motor Values: ");
    // output = float(map(long(output), -3500, 3500, -10, 10));


    leftMotor = MOTOR_SPEED - output;
    rightMotor = MOTOR_SPEED + output;
    
    Serial.print(leftMotor);
    Serial.print(",");
    Serial.println(rightMotor);
    leftMotor = constrain(leftMotor, -90, 90);
    rightMotor = constrain(rightMotor, -90, 90);

    previousMillis = currentMillis;
    prevError = error;
    numIterations++;
    commandMotors();
    delay(50);
  }
  currentState = AIMING;
  leftMotor = 0;
  rightMotor = 0;
  commandMotors();
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  currentState = INIT;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){18, 20, 21, 22, 23, 16, 15, 14}, SensorCount);
  calibrateSensors();

  qtr.setEmitterPin(4); //can get away with a single emitter pin providing power to both emitters
   QTRReadMode::On; //emitters on measures active reflectance instead of ambient light levels, better becasue the ambient light level will change as the robot moves around the board but the reflectance levels will not
}

void loop() {

  distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
  distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  linePosition = qtr.readLineBlack(sensorValues);
  repairLineSensors();
  // Every sensor reading under 300 is a noisy and messes up the lineposition measurement, so this for loop filters it out
  for (int i=0; i<= 7; i++){
    if (sensorValues[i] <300){
        sensorValues[i]=0;     
      }
  }
  
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChar, receivedChars);
    parseData();
    newData = false;
  }

  publishSensorData();
  switch (currentState) {
    case INIT:
      /* code */
      break;
    case DEAD_RECKONING:
      lastSeenWallDistance = flt_infinity;

      faceWallPerpendicularly();

      distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
      distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);

      publishSensorData();
      Serial.println("Facing line now");
      delay(5000);

      if (distanceFront < START_BOX_X) {  // 45.72 cm
        // we are facing the backwall
        wallBeingFaced = BACKWARD;

        Serial.println("Facing the BACKWALL");
        delay(2000);

        if (distanceRight > FIELD_Y / 2) {
          // we are on the left side of the field therefore turn right
          Serial.println("We are on the left side of the line");
          delay(2000);

          distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
          distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
          lastSeenWallDistance = distanceFront;  // reset for next use
          dr_turnRight();
          delay(1000);
          turnTowardRightWall();
          currentState = LINE_FOLLOWING;

        } else {
          // we are on the right side of the field therefore turn left
          Serial.println("We are on the Right side of the line");
          delay(2000);
          distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
          distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
          lastSeenWallDistance = distanceFront;  // reset for next use
          dr_turnLeft();
          delay(1000);
          turnTowardLeftWall();
          currentState = LINE_FOLLOWING;
        }
      } else if (lastSeenWallDistance < (FIELD_Y / 2 + START_BOX_Y / 2)) {  //
        // we are facing one of the side walls

        Serial.println("Facing the SIDEWALLL!");
        delay(2000);

        distanceRight > START_BOX_Y ? wallBeingFaced = RIGHT : wallBeingFaced = LEFT;
        if (distanceFront > FIELD_X / 2) {
          // we are behind the midpoint therefore just drive forward towards the line
          currentState = LINE_FOLLOWING;
        } else {
          // we are facing the side wall and in front of the midpoint therefore turn around
          dr_turnAround(wallBeingFaced);
          stopMotors();
          currentState = LINE_FOLLOWING;
        }

      } else {

        Serial.println("Facing the WONDERWALL!");
        delay(2000);

        // we are facing the front wall
        wallBeingFaced = FORWARD;
        if (distanceRight > (FIELD_Y / 2)) {

          // we are on the left side of the line therefore turn right
          Serial.println("We are on the left side!");
          delay(2000);

          distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
          distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
          lastSeenWallDistance = distanceFront;  // reset for next use
          dr_turnRight();
          delay(1000);
          turnTowardLeftWall();
          currentState = LINE_FOLLOWING;
        } else {
          // we are on the left side of the field therefore turn right
          Serial.println("We are on the right side!");
          delay(2000);

          distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
          distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
          lastSeenWallDistance = distanceFront;  // reset for next use
          dr_turnLeft();
          delay(1000);
          turnTowardLeftWall();
          currentState = LINE_FOLLOWING;
          stopMotors();
        }
        currentState = LINE_FOLLOWING;
      }

      break;
    case LINE_FOLLOWING:
      /* code */
      handleLineFollowing();
      break;
    case AIMING:
      /* code */
      break;
    case SHOOTING:
      /* code */
      break;
    default:
      break;
  }
  commandMotors();
  delay(20);
}

void commandMotors() {

  //read the documentation for the functions that drive the motors in the astar library
  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
  //uncomment to drive motors
}

void setCurrentState(RobotStates state) {
  currentState = state;
}
