#include <signal.h>
#include "tachikoma.h"
#include "xboxctrl.h"

static int exitnow;

void stopme(int signo) {
  exitnow = 1;
}

int main(int argc, char *argv[]) {
  xboxctrl_t ctrl;
  tachikoma *robot;
  char *msg;

  signal(SIGINT, stopme);

  // connect to everything
  xboxctrl_connect(&ctrl);
  if (!ctrl.connected) {
    printf("ERROR: cannot connect to xbox ctrl\n");
    return 1;
  } else {
    printf("Connected to XBOX Controller!\n");
  }
  robot = new tachikoma();
  if (!robot->connected()) {
    printf("ERROR: cannot connect to robot\n");
    xboxctrl_disconnect(&ctrl);
    delete robot;
    return 1;
  }

  while (!exitnow) {
    xboxctrl_update(&ctrl);
    robot->base[1].x = (double)(ctrl.LEFT - ctrl.RIGHT);
    robot->base[1].y = (double)(ctrl.Y - ctrl.X);
    robot->base[1].z = (double)(ctrl.B - ctrl.A);
    robot->base[0].y = ctrl.LJOY.y;
    robot->base[0].yaw = -ctrl.RJOY.x;
    robot->send();
    msg = robot->read_manual(TACHI_NE_DEVID);
    if (msg) {
      printf("read: %s\n", msg);
    }
    msg = robot->read_manual(TACHI_NE_WHEEL_DEVID);
    if (msg) {
      printf("read: %s\n", msg);
    }
  }

  memset(robot->base, 0, sizeof(pose3d_t) * 2);
  memset(robot->arm, 0, sizeof(pose3d_t) * 2);
  robot->send();
  delete robot;
  xboxctrl_disconnect(&ctrl);

  return 0;
}
