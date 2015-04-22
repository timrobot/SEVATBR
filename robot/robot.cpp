#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "tbr.h"
#include "tachikoma.h"

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
        return ((tbr_t *)bot)->connected;
      }
      break;

    case TACHIKOMA:
      bot = (void *)(new tachikoma());
      if (!((tachikoma *)bot)->connected()) {
        delete (tachikoma *)bot;
        bot = NULL;
        printf("[ERROR] Could not connect all the devices on the tachikoma!\n");
        return -1;
      } else {
        return (tachikoma *bot)->numconnected();
      }

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

    case TACHIKOMA:
      if (bot) {
        delete ((tachikoma *)bot);
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
  int forward;
  int rotate;
  tbr_t *tbr;
  tachikoma * t;

  switch (currid) {
    case STANDARD_OUT:
      printf("[STDOUT] arm->x: %f, arm->y: %f, arm->z: %f\n"
          "arm->yaw: %f, arm->pitch: %f, arm->roll: %f\n"
          "base->x: %f, base->y: %f, base->z: %f\n"
          "base->yaw: %f, base->pitch: %f, base->roll: %f\n\n",
          arm->x, arm->y, arm->z,
          arm->yaw, arm->pitch, arm->roll,
          base->x, base->y, base->z,
          base->yaw, base->pitch, base->roll);
      break;

    case TENNIS_BALL_ROBOT:
      tbr = (tbr_t *)bot;
      forward = (base->y > 0.0) - (base->y < 0.0);
      rotate = (base->yaw > 0.0) - (base->yaw < 0.0);
      tbr->left = forward - rotate;
      tbr->left = (tbr->left > 1) ? 1 :
          ((tbr->left < -1) ? -1 : tbr->left);
      tbr->right = forward + rotate;
      tbr->right = (tbr->right > 1) ? 1 :
          ((tbr->right < -1) ? -1 : tbr->right);
      tbr->arm = (arm->pitch > 0.0) - (arm->pitch < 0.0);
      tbr->claw = (arm->yaw > 0.0) - (arm->yaw < 0.0);
      tbr_send(tbr);
      break;

    case TACHIKOMA:
      t = (tachikoma *)bot;
      t->update(base[0], base[1], arm[0], arm[1]);
      break;

    default:
      return -1;
  }
  return 0;
}

/** Have the robot sense the world around it
 *  @return sensor list ending with a NULL, else NULL
 */
pose3d_t **robot::sense(void) {
  return NULL;
}
