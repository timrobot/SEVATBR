#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "tbr.h"

static void *bot;
static uint32_t currid;

/** Initialize the robot to some id
 *  which specifies the device
 *  @param robotid
 *    the id of the device
 *  @return 0 on success, -1 otherwise
 */
int robot::set(uint32_t robotid) {
  if (currid != NO_ROBOT) {
    return -1;
  }
  switch (robotid) {
    case STANDARD_OUT:
      printf("[STDOUT] Robot set\n");
      break;

    case TENNIS_BALL_ROBOT:
      bot = (void *)(new tbr_t);
      if (tbr_connect((tbr_t *)bot) == -1) {
        delete (tbr_t *)bot;
        bot = NULL;
        printf("[ERROR] Could not get all arduinos! Recommend exiting.\n");
        return -1;
      } else {
        currid = robotid;
        return ((tbr_t *)bot)->connected;
      }
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
int robot::unset(void) {
  switch (currid) {
    case STANDARD_OUT:
      printf("[STDOUT] Robot unset.\n");
      break;

    case TENNIS_BALL_ROBOT:
      if (bot) {
        tbr_disconnect((tbr_t *)bot);
        delete (tbr_t *)bot;
      }
      break;

    default:
      return -1;
  }
  currid = NO_ROBOT;
  bot = NULL;
  return 0;
}

/** Move the robot by some velocity or to some position
 *  @param base
 *    either a direction or velocity (base)
 *  @param arm
 *    either a direction or velocity (arm)
 *  @return 0 on success, -1 otherwise
 */
int robot::move(pose3d_t *base, pose3d_t *arm) {
  double forward;
  double rotate;
  tbr_t *tbr;

  switch (currid) {
    case 0x45:
    //case STANDARD_OUT:
      printf("[STDOUT] "
          "arm->yaw: %f, arm->pitch: %f, "
          "base->y: %f, base->yaw: %f\n\n",
          arm->yaw, arm->pitch,
          base->y, base->yaw);
      break;

    case TENNIS_BALL_ROBOT:
      tbr = (tbr_t *)bot;
      forward = base->y;
      rotate = base->yaw;
      tbr->left = forward - rotate;
      tbr->right = forward + rotate;
      tbr->arm = arm->pitch;
      tbr->claw = arm->yaw;
      tbr_send(tbr);
      break;

    default:
      return -1;
  }
  return 0;
}

/** Have the robot sense the world around it
 *  @return sensor list ending with a NULL, else NULL
 */
pose3d_t *robot::sense(void) {
  pose3d_t *sensor_data;
  tbr_t *tbr;

  switch (currid) {
    case STANDARD_OUT:
      sensor_data = (pose3d_t *)calloc(4, sizeof(pose3d_t));
      return sensor_data;

    case TENNIS_BALL_ROBOT:
      tbr = (tbr_t *)bot;
      tbr_recv(tbr);
      // memory leak:
      sensor_data = (pose3d_t *)calloc(4, sizeof(pose3d_t));
      sensor_data[0].y = tbr->sonar[0];
      sensor_data[1].y = tbr->sonar[1];
      sensor_data[2].y = tbr->sonar[2];
      return sensor_data;

    default:
      return NULL;
  }
}
