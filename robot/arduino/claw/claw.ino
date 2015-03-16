#include <Servo.h>
#include <string.h>
#define DEV_ID  3
#define CLAW_L  9 
#define CLAW_R 10

Servo claw_l;
Servo claw_r;
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

void setclaw(int vel) {
  vel = limit(vel, -180, 180);
  claw_l.write(-vel);
  claw_r.write(vel);
}

void stopclaw() {
  setclaw(0);
  //Serial.println("stopclaw");
}

void openclaw() {
  stopclaw();
  delay(50);
  setclaw(180);
  //Serial.println("openclaw");
}

void closeclaw() {
  stopclaw();
  delay(50);
  setclaw(-180);
  //Serial.println("closeclaw");
}

void setup() {
  claw_l.attach(CLAW_L);
  claw_r.attach(CLAW_R);
  pinMode(13, OUTPUT);
  
  Serial.begin(38400);
  stopclaw();
  digitalWrite(13, HIGH);
  time = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case ' ':
        stopclaw();
        break;
      case 'o':
        openclaw();
        break;
      case 'k':
        closeclaw();
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
