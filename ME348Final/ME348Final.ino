#include <AStar32U4Motors.h>

enum MotorStates {
  INIT,
  DEAD_RECKONING,
  LINE_FOLLOWING,
  AIMING,
  SHOOTING
};

const int trigPinFront = 11;
const int echoPinFront = 10;

const int trigPinRight = 9;
const int echoPinRight = 8;

#define Kp 0
#define Ki 0
#define Kd 0

#define SPEED_SOUND 0.0343 // cm/microSeconds
#define NUM_POINTS 200

AStar32U4Motors m;

int leftMotorVelocity = 0;
int rightMotorVelocity = 0;

int leftMotor = 100;
int rightMotor = 0;

MotorStates currentState = INIT;

unsigned long currentTime;

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

void setup() {
  Serial.begin(115200);
  pinMode(trigPinFront, OUTPUT);  
	pinMode(echoPinFront, INPUT);  
  pinMode(trigPinRight, OUTPUT);  
	pinMode(echoPinRight, INPUT);  
  pinMode(LED_BUILTIN, OUTPUT);

  leftMotor = 0;
  rightMotor = 0;

}

void loop() {
  commandMotors();
  currentTime = millis();
  float distanceFront = getUltrasonicReading(trigPinFront, echoPinFront);
  float distanceRight = getUltrasonicReading(trigPinRight, echoPinRight);

  delay(10);
} 



void setMotorVelocity() {}

void commandMotors(){

  //read the documentation for the functions that drive the motors in the astar library
  Serial.print("Motors: ");
  Serial.print(leftMotor);
  Serial.print(" | ");
  Serial.print(rightMotor);
  Serial.println();
  m.setM1Speed(leftMotor);
  m.setM2Speed(rightMotor);
  //uncomment to drive motors
}
