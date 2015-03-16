#include <Servo.h>
#include <string.h>
#define DEV_ID  2
#define ARM_R1  3
#define ARM_L1  5
#define ARM_R2  6
#define ARM_L2  9
#define ARM_R3 10
#define ARM_L3 11

Servo arm_l[3];
Servo arm_r[3];
unsigned long time;
char msg[64];

int limit(int x, int a, int b) {
  if (x > b) {
    return b;
  } else if (x < a) {
    return a;
  } else {
    return x;
  }
}

void setarm(int vel) {
  vel = limit(vel, -180, 180);
  for (int i = 0; i < 3; i++) {
    arm_l[i].write(-vel);
    arm_r[i].write(vel);
  }
}

void stoparm() {
  setarm(0);
  //Serial.println("stoparms");
}

void raisearm() {
  stoparm();
  delay(50);
  setarm(180);
  //Serial.println("raisearm");
}

void lowerarm() {
  stoparm();
  delay(50);
  setarm(-180);
  //Serial.println("lowerarm");
}

void setup() {
  arm_l[0].attach(ARM_L1);
  arm_r[0].attach(ARM_R1);
  arm_l[1].attach(ARM_L2);
  arm_r[1].attach(ARM_R2);
  arm_l[2].attach(ARM_L3);
  arm_r[2].attach(ARM_R3);
  pinMode(13, OUTPUT);
  
  Serial.begin(38400);
  stoparm();
  digitalWrite(13, HIGH);
  time = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case ' ':
        stoparm();
        break;
      case 'p':
        raisearm();
        break;
      case 'l':
        lowerarm();
        break;
      case '\n':
        // do nothing
        break;
      default:
        //Serial.println("err: bad command");
        //Serial.println(c);
        break;
    }
  }
  if (millis() - time > 100) { // 10Hz
    sprintf(msg, "[%d ]", DEV_ID);
    Serial.println(msg);
    time = millis();
  }
}
