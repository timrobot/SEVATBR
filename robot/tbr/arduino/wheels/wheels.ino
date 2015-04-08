#include <string.h>
#define DEV_ID     1
#define WHEEL_RT1 11
#define WHEEL_RT2 10
#define WHEEL_RM1  9
#define WHEEL_RM2  8
#define WHEEL_RB1  7
#define WHEEL_RB2  6
#define WHEEL_LT1 A0
#define WHEEL_LT2 A1
#define WHEEL_LM1 A2
#define WHEEL_LM2 A3
#define WHEEL_LB1 A4
#define WHEEL_LB2 A5

class HBridgeMotor { // HBridge implementation
  public:
    short velocity; // PWM -255 to 255
    char pin[2];
    bool isdigital;
    HBridgeMotor() {
      reset();
      isdigital = false;
    }
    void setdigital(bool d) {
      isdigital = d;
    }
    void write(int v) {
      velocity = v;
      if (!isdigital) {
        int limit = 255;
        if (velocity < -limit) velocity = -limit;
        if (velocity > limit) velocity = limit;
        if (pin[0] == 0 || pin[1] == 0) return;
        if (velocity < 0) {
          analogWrite(pin[0], 0);
          analogWrite(pin[1], -velocity);
        } else {
          analogWrite(pin[1], 0);
          analogWrite(pin[0], velocity);
        }
      } else {
        if (pin[0] == 0 || pin[1] == 0) return;
        if (velocity < 0) {
          digitalWrite(pin[0], LOW);
          digitalWrite(pin[1], HIGH);
        } else if (velocity > 0) {
          digitalWrite(pin[1], LOW);
          digitalWrite(pin[0], HIGH);
        } else {
          digitalWrite(pin[0], LOW);
          digitalWrite(pin[1], LOW);
        }
      }
    }
    int attach(int pin1, int pin2) {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      pin[0] = pin1;
      pin[1] = pin2;
    }
    void reset() {
      pin[0] = 0;
      pin[1] = 0;
      velocity = 0;
    }
};

HBridgeMotor wheel_l[3];
HBridgeMotor wheel_r[3];
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
  left = -limit(left, -255, 255);
  right = limit(right, -255, 255);
  wheel_l[0].write(left);
  wheel_l[1].write(-left);
  wheel_l[2].write(left);
  wheel_r[0].write(right);
  wheel_r[1].write(-right);
  wheel_r[2].write(right);
}

void stopwheels() {
  setwheels(0, 0);
  //Serial.println("stopwheels");
}

void turnleft() {
  stopwheels();
  delay(50);
  setwheels(-255, 255);
  //Serial.println("turnleft");
}

void turnright() {
  stopwheels();
  delay(50);
  setwheels(255, -255);
  //Serial.println("turnright");
}

void forward() {
  stopwheels();
  delay(50);
  setwheels(255, 255);
  //Serial.println("forward");
}

void backward() {
  stopwheels();
  delay(50);
  setwheels(-255, -255);
  //Serial.println("backward");
}

void setup() {
  wheel_l[0].attach(WHEEL_LT1, WHEEL_LT2);
  wheel_l[0].setdigital(true);
  wheel_l[1].attach(WHEEL_LM1, WHEEL_LM2);
  wheel_l[1].setdigital(true);
  wheel_l[2].attach(WHEEL_LB1, WHEEL_LB2);
  wheel_l[2].setdigital(true);

  wheel_r[0].attach(WHEEL_RT1, WHEEL_RT2);
  wheel_r[0].setdigital(true);
  wheel_r[1].attach(WHEEL_RM1, WHEEL_RM2);
  wheel_r[1].setdigital(true);
  wheel_r[2].attach(WHEEL_RB1, WHEEL_RB2);
  wheel_r[2].setdigital(true);

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
