#include <Servo.h>

Servo myservo;

#define OPEN 180
#define CLOSE 110

#define POS_ONE 0
#define POS_TWO 90
#define POS_THREE 180

void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  myservo.attach(5);
  myservo.write(POS_TWO);
  Serial.begin(9600);
}

// 40 to open
// 90 to close

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    String data = Serial.readString();
    int angle = data.toInt();
    if (angle == 0) {
    myservo.write(POS_ONE);
    } else if (angle == 1) {
      myservo.write(POS_TWO);
    } else {
      myservo.write(POS_THREE);
    }
    
  }
  delay(10);
}