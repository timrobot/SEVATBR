#include "rawrec.h"
#include <stdio.h>

int main(int argc, char **argv) {
  rawrec_t rr[2];
  start_recording(&rr[0], "s1.raw");
  sleep(1);
  start_recording(&rr[1], "s2.raw");
  sleep(4);
  stop_recording(&rr[0]);
  stop_recording(&rr[1]);
  return 0;
}
