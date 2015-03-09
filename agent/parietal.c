#include "parietal.h"

extern pose3d_t arm;
extern pose3d_t base;

void turnright() {
  memset(&base, 0, sizeof(pose3d_t));
  base.yaw = -1.0;
}

void stoprobot() {
  memset(&base, 0, sizeof(pose3d_t));
  memset(&arm, 0, sizeof(pose3d_t));
}

void dropball() {
  memset(&arm, 0, sizeof(pose3d_t));
  arm.yaw = 1.0;
}

void pickupball() {
  memset(&pose3d_t, 0, sizeof(pose3d_t));
  if (arm_too_high) {
    arm.pitch = -1.0;
  }
  arm.yaw = -1.0;
}
