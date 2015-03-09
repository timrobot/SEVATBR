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
  signal(SIGINT, stop);
  manual_connect();
  manual_enable();

  while (!stopsig) {
    if (manual_new_data()) {
      manual_get_poses(&base, &arm);
      printf("data: %f %f %f %f\n", base.y, base.yaw, arm.pitch, arm.yaw);
    }
  }

  manual_disconnect();
  return 0;
}
