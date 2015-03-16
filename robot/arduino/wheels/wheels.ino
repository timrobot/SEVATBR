#include <Servo.h>
#include <string.h>
#define DEV_ID    1
#define WHEEL_LB  3
#define WHEEL_RB  5
#define WHEEL_LM  6
#define WHEEL_RM  9
#define WHEEL_LT 10
#define WHEEL_RT 11

Servo wheel_l[3];
Servo wheel_r[3];
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

void setwheels(int left, int right) {
  left = -limit(left, -180, 180);
  right = limit(right, -180, 180);
  wheel_l[0].write(left);
  wheel_r[0].write(right);
  wheel_l[1].write(-left);
  wheel_r[1].write(-right);
  wheel_l[2].write(left);
  wheel_r[2].write(right);
}

void stopwheels() {
  setwheels(0, 0);
  //Serial.println("stopwheels");
}

void turnleft() {
  stopwheels();
  delay(50);
  setwheels(-180, 180);
  //Serial.println("turnleft");
}

void turnright() {
  stopwheels();
  delay(50);
  setwheels(180, -180);
  //Serial.println("turnright");
}

void forward() {
  stopwheels();
  delay(50);
  setwheels(180, 180);
  //Serial.println("forward");
}

void backward() {
  stopwheels();
  delay(50);
  setwheels(-180, -180);
  //Serial.println("backward");
}

void setup() {
  wheel_l[0].attach(WHEEL_LT);
  wheel_r[0].attach(WHEEL_RT);
  wheel_l[1].attach(WHEEL_LM);
  wheel_r[1].attach(WHEEL_RM);
  wheel_l[2].attach(WHEEL_LB);
  wheel_r[2].attach(WHEEL_RB);
  pinMode(13, OUTPUT);
  
  Serial.begin(38400);
  stopwheels();
  digitalWrite(13, HIGH);
  time = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case ' ':
        stopwheels();
        break;
      case 'a':
        turnleft();
        break;
      case 's':
        backward();
        break;
      case 'w':
        forward();
        break;
      case 'd':
        turnright();
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
