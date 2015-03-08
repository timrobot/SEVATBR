#include "parietal.h"

extern pose3d_t arm_signal;
extern pose3d_t base_signal;

void turnright() {
  memset(&ctrlout, 0, sizeof(ctrlout));
  ctrlout.yaw = 1.0;
}

void stoprobot() {
  memset(&ctrlout, 0, sizeof(ctrlout));
}

void dropball() {
  memset(&ctrlout, 0, sizeof(ctrlout));
  ctrlout.arms->joint->next->output.open = 1.0;
}

void pickupball() {
  memset(&ctrlout, 0, sizeof(ctrlout));
  ctrlout.arms->joint.rotate = 1.0;
  ctrlout.arms->joint->next.rotate = -1.0;
  sleep(1);
  ctrlout.arms->joint->next->output.open = -1.0;
}
