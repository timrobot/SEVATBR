#include <pthread.h>
#include "speech.h"
#include "visual.h"
#include "coord.h"
#include "parietal.h"

static pthread_t conciousness;
static int end_conciousness;

typedef struct info {
  speech_signal_t ss;
} info_t;

struct _ctrl {
  pose3d_t 
} ctrl;

void wakeup() {
  start_speech_signals();
  start_visual();
  set_objects(VISUAL_TENNIS_BALL, VISUAL_BASKET, NULL);
  pthread_create(&conciousness, NULL, concious_thought, NULL);
}

void *concious_thought(void *information) {
  info_t *info = (info_t *)information;
  enum goal_states { go, ret, stop };
  enum action_states {
    find_ball,
    goto_ball,
    pickup_ball,
    find_basket,
    goto_basket,
    drop_ball
  };
  int goal = 0;
  int action = 0;
  int hasball = 0;
  point_t *basket_loc;
  int bloclen = 0;
  point_t *tennis_ball_locs;
  int tloclen = 0;
  int i;

  // get voice command
  get_speech_signal(&info->ss);
  if (info->ss.go) {
    goal = goal_states.go;
  } else if (info->ss.ret) {
    goal = goal_states.ret;
  } else if (info->ss.stop) {
    goal = goal_states.stop;
  }

  // finite state automata
  memset(&ctrlout, 0, sizeof(ctrlout));
  switch (goal) {
    case goal_states.go:
      if (hasball) {
        if (bloclen == 0) {
          // policy: rotate around until found
          turnright();
        } else {
          if (basket_loc[0].z < basket_tolerance) {
            dropball();
          } else {
            gotobasket(&basket_loc[0]);
          }
        }
      } else {
        if (tloclen == 0) {
          // policy: torate around until found
          turnright();
        } else {
          if (tennis_ball_loc[0].z < ball_tolerence) {
            pickupball();
          } else {
            gotoball(&tennis_ball_loc[0]);
          }
        }
      }
      break;
    case goal_states.ret:
      if (bloclen == 0) {
        turnright();
      } else {
        if (baseket_loc[0].z < basket_tolerance) {
          dropball();
        } else {
          gotobasket(&basket_loc[0]);
        }
      }
      break;
    case goal_states.stop:
      stoprobot();
      break;
    default:
      break;
  }
}

void gotosleep() {
  end_conciousness = 1;
  pthread_join(conciousness, NULL);
  stop_visual();
  stop_speech_signals();
}
