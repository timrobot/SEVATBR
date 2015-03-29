#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "agent.h"
#include "tbd.h"

static int agent_enabled;
static int view[2];
static struct timeval last_signal;

int agent_create(int type) {
  if (type != AGENT_SIMPLE) {
    return -1;
  } else {
    tbd_start_detecting();
    return 0;
  }
}

void agent_enable(void) {
  agent_enabled = 1;
}

void agent_disable(void) {
  agent_enabled = 0;
}

void agent_destroy(void) {
  agent_disable();
  tbd_stop_detecting();
}

void agent_get_poses(pose3d_t *base, pose3d_t *arm) {
  if (agent_enabled) {
    int newdata;
    newdata = tbd_get_coord(&view[0], &view[1]);
    if (!newdata) {
      struct timeval currtime;
      long diff;
      gettimeofday(&currtime, NULL);
      diff = (currtime.tv_usec - last_signal.tv_usec) + (currtime.tv_sec - last_signal.tv_sec) * 1000000;
      if (diff >= 1000000) {
        memset(base, 0, sizeof(pose3d_t));
        memset(arm, 0, sizeof(pose3d_t));
        gettimeofday(&last_signal, NULL);
      }
    } else {
      int yaw_thresh[2] = { -20, 20 };
      int pitch_thresh[2] = { 0, 220 };
      //int y_thresh = 8;
      arm->yaw = 1.0;

      if (view[0] > yaw_thresh[1] || view[0] < yaw_thresh[0]) {
        base->y = 0.0;
        if (view[0] > yaw_thresh[1]) {
          base->yaw = -1.0;
        } else {
          base->yaw = 1.0;
        }
      } else {
        base->y = 1.0;
        base->yaw = 0.0;
      }

      if (view[1] > pitch_thresh[1] || view[1] < pitch_thresh[0]) {
        if (view[1] > pitch_thresh[1]) {
          arm->pitch = -1.0;
        } else {
          arm->pitch = 0.0;//1.0;
        }
      } else {
        arm->pitch = 0.0;
      }
      gettimeofday(&last_signal, NULL);
    }
  } else {
    printf("agent not enabled\n");
    memset(base, 0, sizeof(pose3d_t));
    memset(arm, 0, sizeof(pose3d_t));
  }
}
