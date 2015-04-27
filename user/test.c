#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "user.h"

static int stopsig;

void stop(int signo) {
  stopsig = 1;
}

int main() {
  pose3d_t base;
  pose3d_t arm;
  user_connect(USER_SRVR);
  user_enable();

  while (!stopsig) {
    user_get_poses(&base, &arm);
    printf("forward: %f, turn: %f, arm: %f, claw: %f\n",
        base.y, base.yaw, arm.pitch, arm.yaw);
  }

  user_disconnect();
  return 0;
}
