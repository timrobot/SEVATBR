#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "manual.h"

static int stopsig;

void stop(int signo) {
  stopsig = 1;
}

int main() {
  pose3d_t base;
  pose3d_t arm;
  manual_connect(MNL_SRVR);
  manual_enable();

  while (!stopsig) {
    manual_get_poses(&base, &arm);
    printf("forward: %f, turn: %f, arm: %f, claw: %f\n",
        base.y, base.yaw, arm.pitch, arm.yaw);
  }

  manual_disconnect();
  return 0;
}
