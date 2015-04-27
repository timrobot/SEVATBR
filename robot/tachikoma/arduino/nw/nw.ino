#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leg1 = AFMS.getMotor(1);
Adafruit_DCMotor *leg2 = AFMS.getMotor(2);
Adafruit_DCMotor *arm1 = AFMS.getMotor(3);
Adafruit_DCMotor *arm2 = AFMS.getMotor(4);

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  leg1->setSpeed(0);
  leg2->setSpeed(0);
  arm1->setSpeed(100);
  arm2->setSpeed(100);
  
  leg1->run(FORWARD);
  leg2->run(FORWARD);
  arm1->run(FORWARD);
  arm2->run(FORWARD);
}
