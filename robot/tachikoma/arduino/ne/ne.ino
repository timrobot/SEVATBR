#include <string.h>
#define DEV_ID     2
#define WHEEL_1    A4
#define WHEEL_2    A5
#define BOTTOM_1   10
#define BOTTOM_2    9
#define TOP_1      12
#define TOP_2      11
#define ROTATE1_1   5
#define ROTATE1_2   6
#define ROTATE2_1   8
#define ROTATE2_2   7
#define BTMENC_1  A2
#define BTMENC_2  A3
#define TOPENC_1  4
#define TOPENC_2  3

class QuadEncoder {
  public:
    long pos;
    bool reversed; // set
    char pin[2];
    QuadEncoder() {
      reset();
    }
    int attach(int pin1, int pin2) {
      pinMode(pin1, INPUT);
      pinMode(pin2, INPUT);
      pin[0] = pin1;
      pin[1] = pin2;
      pin_state[0] = digitalRead(pin[0]);
      pin_state[1] = digitalRead(pin[1]);
    }
    int read() {
      update();
      return pos;
    }
    void reset() {
      pin[0] = 0;
      pin[1] = 0;
      pos = 0;
      velocity = 1; // velocity can either be 1 or -1
      reversed = false;
      pin_state[0] = 0;
      pin_state[1] = 0;
    }
  private:
    void update() {
      if (pin[0] == 0 || pin[1] == 0)
        return;
      // FSA : reg :: 00 01 11 10
      //     : rev :: 00 10 11 01
      char new_state[2] = {
        digitalRead(pin[1]) == HIGH,
        digitalRead(pin[0]) == HIGH
      };
      char delta_state[2] = {
        new_state[1] != pin_state[1],
        new_state[0] != pin_state[0]
      };
      if (delta_state[1] && delta_state[0]) {
        pos += velocity * 2 * (reversed ? -1 : 1);
      } else if (delta_state[1]) {
        velocity = new_state[0] == new_state[1] ? -1 : 1;
        pos += velocity * (reversed ? -1 : 1);
      } else if (delta_state[0]) {
        velocity = new_state[0] == new_state[1] ? 1 : -1;
        pos += velocity * (reversed ? -1 : 1);
      }
      pin_state[0] = new_state[0];
      pin_state[1] = new_state[1];
    }
    char pin_state[2];
    long velocity;  // estimated
};

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

HBridgeMotor wheel;
HBridgeMotor bottom;
HBridgeMotor top;
HBridgeMotor rotate1;
HBridgeMotor rotate2;
int W, B, T, R;

QuadEncoder btmenc;
QuadEncoder topenc;

const int bufsize = 256;
const int safesize = bufsize / 2;
char buf[bufsize];
char msg[bufsize];
char wbuf[safesize];
unsigned long msecs;
char numbuf[4];

int limit(int x, int a, int b) {
  if (x > b) {
    return b;
  } else if (x < a) {
    return a;
  } else {
    return x;
  }
}

void setleg(int w, int b, int t, int r) {
  w = limit(w, -255, 255);
  b = limit(b, -255, 255);
  t = limit(t, -255, 255);
  r = limit(r, -255, 255);
  wheel.write(w);
  bottom.write(b);
  top.write(t);
  rotate1.write(r);
  rotate2.write(r);
}

void setup() {
  wheel.attach(WHEEL_1, WHEEL_2);
  bottom.attach(BOTTOM_1, BOTTOM_2);
  top.attach(TOP_1, TOP_2);
  rotate1.attach(ROTATE1_1, ROTATE1_2);
  rotate2.attach(ROTATE2_1, ROTATE2_2);

  wheel.setdigital(true);
  bottom.setdigital(true);
  top.setdigital(true);
  rotate1.setdigital(true);
  rotate2.setdigital(true);

  btmenc.attach(BTMENC_1, BTMENC_2);
  topenc.attach(TOPENC_1, TOPENC_2);

  pinMode(13, OUTPUT);

  Serial.begin(38400);
  setleg(0, 0, 0, 0);
  digitalWrite(13, HIGH);
  msecs = millis();
}

void loop() {
  int nbytes = 0;
  if ((nbytes = Serial.available())) {
    // read + attach null byte
    int obytes = strlen(buf);
    Serial.readBytes(&buf[obytes], nbytes);
    buf[nbytes + obytes] = '\0';

    // resize just in case
    if (strlen(buf) > safesize) {
      memmove(buf, &buf[strlen(buf) - safesize], safesize);
      buf[safesize] = '\0'; // just in case
    }

    // extract possible message
    char *s, *e;
    if ((e = strchr(buf, '\n'))) {
      e[0] = '\0';
      if ((s = strrchr(buf, '['))) {
        // CUSTOMIZE
        sscanf(s, "[%d %d %d %d]", &W, &R, &T, &B);
        W = limit(W * 255, -255, 255);
        T = limit(T * 255, -255, 255);
        B = limit(B * 255, -255, 255);
        R = limit(R * 255, -255, 255);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  if (W != 0 || B != 0 || T != 0 || R != 0) {
    setleg(0, 0, 0, 0);
  }
  setleg(W, B, T, R);

  btmenc.read();
  topenc.read();

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d]\n", DEV_ID, btmenc.read(), topenc.read());
    Serial.print(wbuf);
    msecs = millis();
  }
}
