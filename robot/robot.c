#include "tbr.h"
#include "robot.h"

static uint32_t currid;
/* Currently supported devices */
static tbr_t tbr;

/** Initialize the robot to some id
 *  which specifies the device
 *  @param robotid
 *    the id of the device
 */
int robot_set(uint32_t robotid) {
  // disconnect current robot
  robot_unset();

  switch (robotid) {
    case TENNIS_BALL_ROBOT:
      // try to connect to the device...
      // if it doesn't connect, assume that it is safe
      // and just do nothing
      if (tbr_connect(&tbr) != -1) {
        currid = robotid;
        return 0;
      } else {
        printf("Could not get all arduinos. By policy, exiting\n");
      }
      break;
    default:
      break;
  }
  return -1;
}

/** Give the robot some input
 *  @param controls
 *    the controls for the robot
 *  @return 0 on success, -1 otherwise
 */
int robot_set_controls(robotctrl_t *controls) {
  // Policy: enact turns in higher priority than motion
  switch (robotid) {
    case TENNIS_BALL_ROBOT:
      if (yaw > 0) {
        tbr_turn_right(&tbr);
      } else if (yaw < 0) {
        tbr_turn_left(&tbr);
      } else if (y > 0) {
        tbr_move_forward(&tbr);
      } else if (y < 0) {
        tbr_move_backward(&tbr);
      } else {
        tbr_stop_wheels(&tbr);
      }
      return 0;
      break;
    default:
      break;
  }
  return -1;
}

/** Get the observed state of the robot
 *  @param state
 *    the state variable to modify
 *  @return 0 on success, -1 otherwise
 */
int robot_get_state(robotctrl_t *state) {
  switch (robotid) {
    case TENNIS_BALL_ROBOT:
      break;
    default:
      break;
  }
  return -1;
}

/** Remove the robot from the known space
 *  @return 0 on success, -1 otherwise
 */
int robot_unset(void) {
  switch (currid) {
    case TENNIS_BALL_ROBOT:
      tbr_disconnect(&tbr);
      return 0;
      break;
    default:
      return -1;
      break;
  }
  currid = 0;
  return 0;
}
