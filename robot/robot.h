#ifndef robot_h
#define robot_h

#include <stdint.h>
#define TENNIS_BALL_ROBOT   0x00000002

#ifdef __cplusplus
extern "C" {
#endif

typedef struct robotctrl {
  uint8_t x;
  uint8_t y;
  uint8_t z;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
} robotctrl_t;

typedef struct robotstate_t {
  int nil;
} robotstate_t;

// You can only control 1 robot at a time.
void robot_set(uint32_t robotid);
void robot_set_controls(robotctrl_t *controls);
void robot_get_state(robotstate_t *state);
void robot_unset(void);

#ifdef __cplusplus
}
#endif

#endif
