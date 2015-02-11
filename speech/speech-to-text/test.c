#include <stdio.h>
#include <signal.h>
#include "speech_signal.h"

static unsigned char stopsig;

void stop(int param) {
  stopsig = 1;
}

int main(int argc, char **argv) {
  signal(SIGINT, stop);

  start_speech_signals();
  speech_signal_t sigframe;

  while (!stopsig) {
    get_signal(&sigframe);
//    printf("none: \t%d\ngo: \t%d\n, stop: \t%d\n, fetch: \t%d\n, ret: \t%d\n",
//        sigframe.none, sigframe.go, sigframe.stop, sigframe.fetch, sigframe.ret);
  }

  stop_speech_signals();
  return 0;
}
