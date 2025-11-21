#include <AStar32U4Motors.h>
#include <float.h>
#include <limits.h>

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

  Serial.print(distanceFront);
  Serial.print(",");
  Serial.print(distanceRight);

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
  dr_lastDirection = LEFT;
  leftMotor = -100;
  rightMotor = 100;
  commandMotors();
}

void dr_turnRight() {  // make it turn 90 degrees
  dr_lastDirection = RIGHT;
  leftMotor = 100;
  rightMotor = -100;
  commandMotors();
}

void dr_turnAround(Directions bias) {
  leftMotor = 150;
  rightMotor = -150;
  commandMotors();
  delay(1000);  // adjust this delay to achieve a 180-degree turn
  stopMotors();
  // TODO: Add the bias
  if (bias == LEFT) {
    leftMotor = -50;
    rightMotor = 50;
    delay(300);
  } else {
    dr_lastDirection = RIGHT;
    leftMotor = 50;
    rightMotor = -50;
    delay(300);
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

  while (!facingWall) {
    // publishSensorData();
    // facingWall = faceWallPerpendicularly();
    distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
    distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
    unsigned long currentMillis = millis();
    Serial.print("Last Direction: ");
    Serial.println(dr_lastDirection);

    // find the local minimum distance to the wall
    if (distanceFront < lastSeenWallDistance) {
      lastSeenWallDistance = distanceFront;
      if (distanceFront < minDistanceSeen) {
        lastMinChanged = millis();
        minDistanceSeen = distanceFront;
      }
      dr_turn(dr_lastDirection);
    } else {
      // if the distance starts increasing, turn the other way

      if (currentMillis > lastMinChanged + 5000) {
        // if it's been a while since the last minimum, assume we're done
        Serial.println("Found the closest wall!");
        stopMotors();
        delay(1000);
        return;
      }

      dr_turn(-1 * dr_lastDirection);
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
}

void loop() {

  distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);
  distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);

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
