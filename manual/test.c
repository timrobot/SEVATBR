#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "controller.h"

static int stopsig;

void stop(int signo) {
  stopsig = 1;
}

int main() {
  controller_t ctrl;
  controller_connect(&ctrl);
  for (;;) {
    printf("%d ", ctrl.A);
    printf("%d ", ctrl.B);
    printf("%d ", ctrl.X);
    printf("%d ", ctrl.Y);
    printf("%d ", ctrl.UP);
    printf("%d ", ctrl.DOWN);
    printf("%d ", ctrl.LEFT);
    printf("%d ", ctrl.RIGHT);
    printf("%d ", ctrl.LB);
    printf("%d ", ctrl.RB);
    printf("%d ", ctrl.LB2);
    printf("%d ", ctrl.RB2);
    printf("%d ", ctrl.START);
    printf("%d ", ctrl.SELECT);
    printf("%d ", ctrl.HOME);
    printf("%d ", ctrl.RB2);
    printf("%d ", ctrl.LJOY.pressed);
    printf("%d ", ctrl.RJOY.pressed);
    printf("\n");
  }
  controller_disconnect(&ctrl);
  return 0;
}
