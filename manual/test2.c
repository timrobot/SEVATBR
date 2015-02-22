#include <stdio.h>
#include <signal.h>
#include "manual.h"

static int stop_sig;
void stop(int signo) {
  stop_sig = 1;
}

int main(int argc, char *argv[]) {
  signal(SIGINT, stop);
  manual_t mnl;
  if (manual_connect(&mnl) == 0) {
    while (!stop_sig) {
      robotctrl_t *ctrl;
      if ((ctrl = manual_get(&mnl))) {
        printf("%d %d %d %d %d %d\n",
            ctrl->x,
            ctrl->y,
            ctrl->z,
            ctrl->yaw,
            ctrl->pitch,
            ctrl->roll);
      }
    }
    manual_disconnect(&mnl);
  }
  return 0;
}
