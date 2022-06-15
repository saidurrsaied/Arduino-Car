#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h> 
#include <SoftwareSerial.h>


#define TRIG_PIN A3
#define ECHO_PIN A2
#define MAX_DISTANCE 400
#define LEFT_ANG 135
#define RIGHT_ANG 45
#define FRONT_ANG 90

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
SoftwareSerial Bluetooth(14, 15);

AF_DCMotor leftFRONT(1);
AF_DCMotor leftBACK(2);
AF_DCMotor rightFRONT(4);
AF_DCMotor rightBACK(3);
Servo sensorServo;

int forwardSPEED = 80 ;
int speedADJUSTMENT = 50 ; 
int leftSPEED= forwardSPEED;
int rightSPEED= leftSPEED + speedADJUSTMENT ;
int turningDELAY = 50;
int slow = 70;
int fast = 50;
int distanceFRONT;
int distanceLEFT;
int distanceRIGHT;
int carDIRECTION = 0;

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);
  sensorServo.attach(10);
  sensorServo.write(FRONT_ANG);




}

void loop() {


distanceFRONT = measureDISTANCE();
  if (distanceFRONT >= 30) {
    goFORWARD();

  }
  else {
    breakCAR();
    goBACKWARD();
    breakCAR();
    sensorServo.write(RIGHT_ANG);
    delay(500);
    distanceRIGHT = measureDISTANCE ();
    delay(500);
    sensorServo.write(LEFT_ANG);
    delay(500);
    distanceLEFT = measureDISTANCE ();
    delay(500);
    sensorServo.write(FRONT_ANG);


    if (distanceRIGHT >= distanceLEFT) {

      turnRIGHT();
      breakCAR();

    }
    else {
      turnLEFT();
      breakCAR();
    }}


}
void goFORWARD() {
  leftFRONT.setSpeed(forwardSPEED);
  leftBACK.setSpeed(forwardSPEED);
  rightFRONT.setSpeed(forwardSPEED);
  rightBACK.setSpeed(forwardSPEED);

  leftFRONT.run(FORWARD);
  leftBACK.run(FORWARD);
  rightFRONT.run(FORWARD);
  rightBACK.run(FORWARD);

}
void goBACKWARD() {
  leftFRONT.setSpeed(forwardSPEED);
  leftBACK.setSpeed(forwardSPEED);
  rightFRONT.setSpeed(forwardSPEED);
  rightBACK.setSpeed(forwardSPEED);

  leftFRONT.run(BACKWARD);
  leftBACK.run(BACKWARD);
  rightFRONT.run(BACKWARD);
  rightBACK.run(BACKWARD);
  delay (500);
}

void breakCAR () {
  leftFRONT.run(RELEASE);
  leftBACK.run(RELEASE);
  rightFRONT.run(RELEASE);
  rightBACK.run(RELEASE);
}

void turnLEFT() {
  
  leftFRONT.setSpeed(leftSPEED);
  leftBACK.setSpeed(leftSPEED);
  rightFRONT.setSpeed(rightSPEED);
  rightBACK.setSpeed(rightSPEED);

  leftFRONT.run(BACKWARD);
  leftBACK.run(BACKWARD);
  rightFRONT.run(FORWARD);
  rightBACK.run(FORWARD);
  delay(turningDELAY);
}

void turnRIGHT() {


  leftFRONT.setSpeed(leftSPEED);
  leftBACK.setSpeed(leftSPEED);
  rightFRONT.setSpeed(rightSPEED);
  rightBACK.setSpeed(rightSPEED);

  leftFRONT.run(FORWARD);
  leftBACK.run(FORWARD);
  rightFRONT.run(BACKWARD);
  rightBACK.run(BACKWARD);
  delay(turningDELAY);

}


int measureDISTANCE () {
  delay (50);
  int distance = sonar.ping_cm();

  if (distance == 0)
  {
    distance = MAX_DISTANCE;
  }
  return distance;

}


void rcDRIVE () {
  if (Bluetooth.available())
    carDIRECTION = Bluetooth.read();
  switch (carDIRECTION) {
    case 48: breakCAR();
     break; 
    case 49: turnLEFT();
     break;
    case 50: turnRIGHT();
      break;
    case 51: goBACKWARD();
       break;
    case 52: goFORWARD();
      break;

  }
}
