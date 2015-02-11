#include "rawrec.h"
#include <stdio.h>

int main(int argc, char **argv) {
  rawrec_t rr;
  start_recording(&rr, "rawsample.raw");
  sleep(5);
  stop_recording(&rr);
  return 0;
}
