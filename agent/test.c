#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include "tbd.h"

static int exit_signal;

void stopprog(int signum) {
  exit_signal = 1;
}

int main(int argc, char *argv[]) {
  signal(SIGINT, stopprog);
  tbd_start_detecting();

  while (!exit_signal) {
    int coord[2];
    int i;
    double avgx = 0.0;
    double avgy = 0.0;
    int xcount = 0;
    int ycount = 0;
    const int hz = 50;
    const int reads = 1000;
    long nanotime = 1000000000 / hz / reads;
    struct timespec waittime;
    waittime.tv_sec = 0;
    waittime.tv_nsec = nanotime;
    for (i = 0; i < reads; i++) {

      if (tbd_get_coord(&coord[0], &coord[1])) {
        if (xcount == 0) {
          avgx = coord[0];
        } else {
          avgx += coord[0];
        }
        if (ycount == 0) {
          avgy = coord[1];
        } else {
          avgy += coord[1];
        }
        xcount++;
        ycount++;
      }
      nanosleep(&waittime, NULL);
    }
    if (xcount == 0 || ycount == 0) {
      printf("not found\n");
    } else {
      avgx = avgx / xcount;
      avgy = avgy / ycount;
      printf("x: %f, y: %f\n", avgx, avgy);
    }
  }

  tbd_stop_detecting();
  return 0;
}
