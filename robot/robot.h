#ifndef robot_h
#define robot_h

#include <stdint.h>
#include "coord.h"
#define NO_ROBOT            0x00000000
#define STANDARD_OUT        0x00000001
#define TENNIS_BALL_ROBOT   0x00000002
#define TACHIKOMA           0x00000003

namespace robot {
  // You can only control 1 robot at a time.
  int set(uint32_t robotid);
  int unset(void);
  int move(pose3d_t *base, pose3d_t *arm);
  pose3d_t **sense(void);
}

#endif
