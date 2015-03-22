#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "tbr.h"

static void *robot;
static uint32_t currid;

/** Initialize the robot to some id
 *  which specifies the device
 *  @param robotid
 *    the id of the device
 */
int robot_set(uint32_t robotid) {
  if (currid != NO_ROBOT) {
    return -1;
  }
  switch (robotid) {

    case STANDARD_OUT:
      printf("Robot set\n");
      break;

    case TENNIS_BALL_ROBOT:
      robot = malloc(sizeof(tbr_t));
      if (tbr_connect((tbr_t *)robot) == -1) {
        printf("Could not get all arduinos. By policy, returning\n");
        return -1;
      }
      break;

    case SIMULATOR:
      // TODO
      break;

    default:
      return -1;
  }
  currid = robotid;
  return 0;
}

/** Remove the robot from the known space
 *  @return 0 on success, -1 otherwise
 */
int robot_unset(void) {
  switch (currid) {

    case STANDARD_OUT:
      printf("Robot unset.\n");
      break;

    case TENNIS_BALL_ROBOT:
      tbr_disconnect((tbr_t *)robot);
      break;

    case SIMULATOR:
      // TODO
      break;

    default:
      return -1;
  }
  currid = NO_ROBOT;
  return 0;
}

/** Move the robot by some velocity or to some position
 *  @param base
 *    either a direction or velocity (base)
 *  @param arm
 *    either a direction or velocity (arm)
 */
int robot_move(pose3d_t *base, pose3d_t *arm) {
  switch (currid) {

    case STANDARD_OUT:
      printf("forward: %f, turn: %f, arm: %f, claw: %f\n",
          base->y, base->yaw, arm->pitch, arm->yaw);
      break;

    case TENNIS_BALL_ROBOT:
      {
        int forward;
        int rotate;
        tbr_t *tbr;

        printf("forward: %f, turn: %f, arm: %f, claw: %f\n",
          base->y, base->yaw, arm->pitch, arm->yaw);

        tbr = (tbr_t *)robot;
        forward = (base->y > 0.0) - (base->y < 0.0);
        rotate = (base->yaw > 0.0) - (base->yaw < 0.0);
        tbr->left = forward - rotate;
        tbr->left = (tbr->left > 1) ? 1 : ((tbr->left < -1) ? -1 : tbr->left);
        tbr->right = forward + rotate;
        tbr->right = (tbr->right > 1) ? 1 : ((tbr->right < -1) ? -1 : tbr->right);
        tbr->arm = (arm->pitch > 0.0) - (arm->pitch < 0.0);
        tbr->claw = (arm->yaw > 0.0) - (arm->yaw < 0.0);
        tbr_send(tbr);
      }
      break;

    case SIMULATOR:
      // TODO
      break;

    default:
      return -1;
  }
  return 0;
}
