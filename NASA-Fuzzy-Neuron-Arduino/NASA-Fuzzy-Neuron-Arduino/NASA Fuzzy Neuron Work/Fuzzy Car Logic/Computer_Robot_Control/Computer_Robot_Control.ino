#include <Servo.h>

Servo servoRight;
Servo servoLeft;

void setup() {
  Serial.begin(9600);
  servoRight.attach(13);
  servoLeft.attach(12);
}

// STATES: STOP, FORWARD, LEFT, RIGHT, BACKWARDS
// AKA     ANY,     W,      A,    D,        S
char currentState = 'X';
void loop() {
  if (Serial.available() > 0) {
    currentState = Serial.read();
    Serial.println(currentState);
  }
  if (currentState == 'w') { // forwards
    servoRight.writeMicroseconds(1200);
    servoLeft.writeMicroseconds(1700);
    Serial.println("FORWARDS");
  }
  else if (currentState == 'a') { // left
    servoRight.writeMicroseconds(1200);
    servoLeft.writeMicroseconds(1200);
    Serial.println("LEFT");
  }
  else if (currentState == 's') { // backwards
    servoRight.writeMicroseconds(1700);
    servoLeft.writeMicroseconds(1200);
    Serial.println("BACKWARDS");
  }
  else if (currentState == 'd') { // right
    servoRight.writeMicroseconds(1700);        
    servoLeft.writeMicroseconds(1700);
    Serial.println("RIGHT");
  }
  else { // STOP
    servoRight.writeMicroseconds(1500);
    servoLeft.writeMicroseconds(1500);
    Serial.println("STOP");
  }
  delay(500);
}
