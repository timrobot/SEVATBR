#ifndef robot_h
#define robot_h

#include <stdint.h>
#include "coord.h"
#define NO_ROBOT            0x00000000
#define STANDARD_OUT        0x00000001
#define TENNIS_BALL_ROBOT   0x00000002
#define SIMULATOR           0x00000003

#ifdef __cplusplus
extern "C" {
#endif

// You can only control 1 robot at a time.
int robot_set(uint32_t robotid);
int robot_unset(void);
int robot_move(pose3d_t *base, pose3d_t *arm);

#ifdef __cplusplus
}
#endif

#endif
