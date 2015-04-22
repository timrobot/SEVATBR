#include <string.h>
#define DEV_ID      6
#define WHEEL_1     9
#define WHEEL_2    10

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
    void attach(int pin1, int pin2) {
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
int W;

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

void setwheel(int w) {
  w = limit(w, -255, 255);
  wheel.write(w);
}

void setup() {
  wheel.attach(WHEEL_1, WHEEL_2);
  
  pinMode(13, OUTPUT);

  Serial.begin(38400);
  digitalWrite(13, HIGH);
  setwheel(0);
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
        sscanf(s, "[%d]", &W);
        W = limit(W, -255, 255);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  if (W != 0) {
    setwheel(0);
  }
  setwheel(W);

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d ]\n", DEV_ID);
    Serial.print(wbuf);
    msecs = millis();
  }
}
