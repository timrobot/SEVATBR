#include <string.h>
#define DEV_ID      2
#define BOTTOM_1    3
#define BOTTOM_2    5
#define TOP_1      10
#define TOP_2      11
#define ROTATE_1    9
#define ROTATE_2    6
#define BTMENC_1   A2
#define BTMENC_2   A3
#define TOPENC_1    4
#define TOPENC_2    2
#define POT        A0

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

HBridgeMotor bottom;
HBridgeMotor top;
HBridgeMotor rotate;
int B, T, R;

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

void setleg(int b, int t, int r) {
  b = limit(b, -255, 255);
  t = limit(t, -255, 255);
  r = limit(r, -255, 255);
  bottom.write(b);
  top.write(t);
  rotate.write(r);
}

void setup() {
  bottom.attach(BOTTOM_1, BOTTOM_2);
  top.attach(TOP_1, TOP_2);
  rotate.attach(ROTATE_1, ROTATE_2);

  //bottom.setdigital(true);
  //top.setdigital(true);
  //rotate1.setdigital(true);
  //rotate2.setdigital(true);

  btmenc.attach(BTMENC_1, BTMENC_2);
  topenc.attach(TOPENC_1, TOPENC_2);
  pinMode(POT, INPUT);

  pinMode(13, OUTPUT);

  Serial.begin(38400);
  setleg(0, 0, 0);
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
        sscanf(s, "[%d %d %d %d]", &R, &T, &B);
        T = limit(T, -255, 255);
        B = limit(B, -255, 255);
        R = limit(R, -255, 255);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  if (B != 0 || T != 0 || R != 0) {
    setleg(0, 0, 0);
  }
  setleg(B, T, R);

  btmenc.read();
  topenc.read();

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d %d]\n", DEV_ID, analogRead(POT), topenc.read(), btmenc.read());
    Serial.print(wbuf);
    msecs = millis();
  }
}
