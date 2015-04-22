#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include "tachikoma.h"
#include "xboxctrl.h"

static int exitnow;

void stopprog(int signum) {
  exitnow = 1;
}

void reset_tachikoma(const tachikoma &robot) {
  robot.write_manual(NW_DEVID, (char *)"[0 0 0]\n");
  robot.write_manual(NE_DEVID, (char *)"[0 0 0]\n");
  robot.write_manual(SW_DEVID, (char *)"[0 0 0]\n");
  robot.write_manual(SE_DEVID, (char *)"[0 0 0]\n");
}

int main(int argc, char *argv[]) {
  xboxctrl_t ctrl;
  int legmode;
  char msg[256];
  int act[3];

  char prevmsg[256];
  int prevlegmode = 0;
  char *readmsg = NULL;

  signal(SIGINT, stopprog);

  // init the controller
  xboxctrl_connect(&ctrl);
  tachikoma robot; // already init'd
  legmode = 0;

  // query for values
  while (!exitnow) {
    if (!robot.connected()) {
      break;
    }

    // 1. update controller
    xboxctrl_update(&ctrl);
    // 2. detect mode
    if (ctrl.UP) {
      reset_tachikoma(robot);
      legmode = NW_DEVID;
    } else if (ctrl.DOWN) {
      reset_tachikoma(robot);
      legmode = NE_DEVID;
    } else if (ctrl.LEFT) {
      reset_tachikoma(robot);
      legmode = SW_DEVID;
    } else if (ctrl.RIGHT) {
      reset_tachikoma(robot);
      legmode = SE_DEVID;
    }

    if (legmode == 0) {
      continue;
    }

    // 3. determine the actuator output
    act[0] = (ctrl.Y - ctrl.X) * 256;
    act[1] = (ctrl.B - ctrl.A) * 256;
    act[2] = (ctrl.RB - ctrl.LB) * 256;

    // 4. send to robot
    sprintf(msg, "[%d %d %d]\n",
        act[0], act[1], act[2]);
    if (strcmp(msg, prevmsg) != 0 ||
        prevlegmode != legmode) {
      printf("writing[%d]: %s", legmode, msg);
      robot.write_manual(legmode, msg);
      prevlegmode = legmode;
      strcpy(prevmsg, msg);
    }

    // 5. read current message
    if ((readmsg = robot.read_manual(legmode))) {
      printf("read[%d]: %s\n", legmode, readmsg);
    }
  }

  printf("Exiting.\n");
  reset_tachikoma(robot);

  // stop everything
  xboxctrl_disconnect(&ctrl);
  return 0;
}
