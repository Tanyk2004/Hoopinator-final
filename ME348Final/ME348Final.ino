#include <AStar32U4Motors.h>

enum RobotStates {
  INIT = 0,
  DEAD_RECKONING = 1,
  LINE_FOLLOWING = 2,
  AIMING = 3,
  SHOOTING = 4
};

const int trigPinFront = 7;
const int echoPinFront = 6;

const int trigPinRight = 11;
const int echoPinRight = 8;

#define Kp 0
#define Ki 0
#define Kd 0

#define SPEED_SOUND 0.0343 // cm/microSeconds
#define NUM_POINTS 200

AStar32U4Motors m;

int leftMotorVelocity = 0;
int rightMotorVelocity = 0;

int leftMotor = 0;
int rightMotor = 0;

RobotStates currentState = INIT;

unsigned long currentTime;
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing
boolean newData = false;
float distanceFront;
float distanceRight;

float getUltrasonicReading(int trigPin, int echoPin) {
	digitalWrite(trigPin, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trigPin, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trigPin, LOW);  

  float duration = pulseIn(echoPin, HIGH);  
  float distance = (duration * SPEED_SOUND)/2;

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
            }
            else {
                receivedChars[ndx] = '\0'; // terminates the string, frankly unsure why I need this
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

void parseData(){
  
  char *strtokIndexer; //doing char * allows strtok to increment across my string properly frankly im not sure why... something to do with pointers that I dont expect students to understand
  
  strtokIndexer = strtok(tempChar,","); //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  leftMotor = atoi(strtokIndexer); //converts strtokIndexer into a int
  strtokIndexer= strtok(NULL, ","); //setting the first input to null causes strtok to continue looking for commas in tempChar starting from where it left off, im not really sure why 
  rightMotor = atoi(strtokIndexer);
  strtokIndexer = strtok(NULL, ",");
  currentState = static_cast<RobotStates>(atoi(strtokIndexer));

}



void setup() {
  Serial.begin(115200);
  pinMode(trigPinFront, OUTPUT);  
	pinMode(echoPinFront, INPUT);  
  pinMode(trigPinRight, OUTPUT);  
	pinMode(echoPinRight, INPUT);  
  

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
  commandMotors();
  
  delay(20);
} 

void setMotorVelocity() {}

void commandMotors(){

  //read the documentation for the functions that drive the motors in the astar library
  m.setM1Speed(leftMotor);
  m.setM2Speed(rightMotor);
  //uncomment to drive motors
}

void setCurrentState(RobotStates state){
  currentState = state;
}
