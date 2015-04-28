#include <vector>
#include <string.h>
#include <stdio.h>
#include "visual.h"
#include "speech.h"
#include "robot.h"
#include "agent.h"

enum agentstates {
  S_IDLE,
  S_FINDBALL,
  S_GOTOBALL,
  S_PICKBALL,
  S_FINDBASKET,
  S_GOTOBASKET,
  S_DROPBASKET
};
enum objstates {
  S_BALL,
  S_BASKET
};
static bool enable;
static pose3d_t agent_base;
static pose3d_t agent_arm;
// state space (2-level decision resolution)
static const char *task;
static int subtask;
static int numballs;
static int visual_detect_type;
static std::vector<pose3d_t> updated_locs;
static std::vector<pose3d_t> ballpos;
static std::vector<pose3d_t> basketpos;
static void conscious_thought(void);
static char *get_speech_command(void);
static double limitf(double x, double min, double max);
static void set_robot(double forward, double left, double raise, double grab);
static void look_for(int detectingstate);
static std::vector<pose3d_t> grab_new_frame_object(int &n);
static int num_balls_in_basket(void);
static std::vector<pose3d_t> get_filtered_ball_positions(void);
static pose3d_t closest_object(std::vector<pose3d_t> locs);
static std::vector<pose3d_t> get_filtered_basket_position(void);
static bool can_pickup(pose3d_t pos);
static bool can_drop(pose3d_t pos);

/** Wake the agent up, start all initial processes
 *  @return 0 on success, -1 on error
 */
int agent::wakeup(void) {
  // start the vision engine(s)
  start_visual();
  set_detection(DETECT_BALL);
  visual_detect_type = S_BALL;
  task = "stop";
  subtask = S_IDLE;
  // start the speech to text listener
  speech::start();
  set_enable(true);
  return 0;
}

/** Make the agent go to sleep, stop all processes
 */
void agent::sleep(void) {
  set_enable(false);
  // stop the vision engine(s)
  stop_visual();
  // stop the speech to text listener
  speech::stop();
}

/** Set the enable for the AI
 *  @param en
 *    true for enable, false for disable
 */
void agent::set_enable(bool en) {
  enable = en;
}

/** Get the poses
 *  @param base
 *    the base struct
 *  @param arm
 *    the arm struct
 *  @return 0
 */
int agent::get_poses(pose3d_t *base, pose3d_t *arm) {
  conscious_thought();
  if (!enable) {
    memset(base, 0, sizeof(pose3d_t));
    memset(arm, 0, sizeof(pose3d_t));
  } else {
    memcpy(base, &agent_base, sizeof(pose3d_t));
    memcpy(arm, &agent_arm, sizeof(pose3d_t));
  }
  return 0;
}

/** Private Functions **/

/** Update the current poses of the robot
 */
static void conscious_thought(void) {
  char *scmd;
  int execute;
  // get a speech command, and place that as
  // the priority task
  if ((scmd = get_speech_command())) {
    task = scmd;
  }

  // FSM (taskgraph)
  execute = 0;
  while (!execute) {
    if (strcmp(task, "fetch") == 0) {
      // transitions
      // TODO: create a counter resolution for infinite loops
      // TODO: pick up more than 1 ball
      switch (subtask) {
        case S_IDLE:
          if (num_balls_in_basket() == 0) {
            subtask = S_FINDBALL;
          } else {
            subtask = S_FINDBASKET;
          }
          break;
        case S_FINDBALL:
          look_for(S_BALL);
          if (num_balls_in_basket() != 0) {
            subtask = S_FINDBASKET;
          } else if ((ballpos = get_filtered_ball_positions()).size() == 0) {
            execute = 1;
          } else {
            subtask = S_GOTOBALL;
          }
          break;
        case S_GOTOBALL:
          if (num_balls_in_basket() != 0) {
            subtask = S_FINDBASKET;
          } else if ((ballpos = get_filtered_ball_positions()).size() == 0) {
            subtask = S_FINDBALL;
          } else if (can_pickup(closest_object(ballpos))) {
            subtask = S_PICKBALL;
          } else {
            execute = 1;
          }
          break;
        case S_PICKBALL:
          if (num_balls_in_basket() != 0) {
            subtask = S_FINDBASKET;
          } else if ((ballpos = get_filtered_ball_positions()).size() == 0) {
            subtask = S_FINDBALL;
          } else if (!can_pickup(closest_object(ballpos))) {
            subtask = S_GOTOBALL;
          } else {
            execute = 1;
          }
          break;
        case S_FINDBASKET:
          look_for(S_BASKET);
          if (num_balls_in_basket() == 0) {
            subtask = S_FINDBALL;
          } else if ((basketpos = get_filtered_basket_position()).size() == 0) {
            execute = 1;
          } else {
            subtask = S_GOTOBASKET;
          }
          break;
        case S_GOTOBASKET:
          if (num_balls_in_basket() == 0) {
            subtask = S_FINDBALL;
          } else if ((basketpos = get_filtered_basket_position()).size() == 0) {
            subtask = S_FINDBASKET;
          } else if (can_drop(closest_object(basketpos))) {
            subtask = S_DROPBASKET;
          } else {
            execute = 1;
          }
          break;
        case S_DROPBASKET:
          if (num_balls_in_basket() == 0) {
            subtask = S_FINDBALL;
          } else if ((basketpos = get_filtered_basket_position()).size() == 0) {
            subtask = S_FINDBASKET;
          } else if (!can_drop(closest_object(basketpos))) {
            subtask = S_GOTOBASKET;
          } else {
            execute = 1;
          }
          break;
        default:
          subtask = S_IDLE;
      }
    } else if (strcmp(task, "return") == 0) {
      switch (subtask) {
        case S_IDLE:
          if ((basketpos = get_filtered_basket_position()).size() != 0 &&
              can_drop(closest_object(basketpos)) &&
              num_balls_in_basket() == 0) {
            execute = 1;
          } else {
            subtask = S_FINDBASKET;
          }
          break;
        case S_FINDBASKET:
          if ((basketpos = get_filtered_basket_position()).size() == 0) {
            execute = 1;
          } else if (!can_drop(closest_object(basketpos)) ||
              num_balls_in_basket() != 0) {
            subtask = S_GOTOBASKET;
          } else {
            subtask = S_IDLE;
          }
          break;
        case S_GOTOBASKET:
          if ((basketpos = get_filtered_basket_position()).size() == 0) {
            subtask = S_FINDBASKET;
          } else if (!can_drop(closest_object(basketpos))) {
            execute = 1;
          } else if (num_balls_in_basket() != 0) {
            subtask = S_DROPBASKET;
          } else {
            subtask = S_IDLE;
          }
          break;
        case S_DROPBASKET:
          if ((basketpos = get_filtered_basket_position()).size() == 0) {
            subtask = S_FINDBASKET;
          } else if (!can_drop(closest_object(basketpos))) {
            subtask = S_GOTOBASKET;
          } else if (num_balls_in_basket() != 0) {
            execute = 1;
          } else {
            subtask = S_IDLE;
          }
          break;
        default:
          subtask = S_IDLE;
      }
    } else if (strcmp(task, "stop") == 0) {
      subtask = S_IDLE;
      execute = 1;
    }
  }

  // move accordingly to the state
  // TODO: do collision detection, stop the robot from moving
  // TODO: on the robot side, make 3 pose3 for sensors
  // TODO: have ultrasonic sensors = inf when blocked by potentiometer
  switch (subtask) {
    case S_IDLE:
      set_robot(0.0, 0.0, 0.0, 0.0);
      break;
    case S_FINDBALL:
      // spin around
      set_robot(0.0, -0.5, 0.0, 0.0);
      break;
    case S_GOTOBALL:
      set_robot(1.0, -ballpos[0].x, -ballpos[0].y, 0.0);
      break;
    case S_PICKBALL:
      set_robot(1.0, -ballpos[0].x, -ballpos[0].y, 1.0);
      break;
    case S_FINDBASKET:
      set_robot(0.0, -0.5, 0.0, 0.0);
      break;
    case S_GOTOBASKET:
      set_robot(1.0, -basketpos[0].x, -basketpos[0].y, 0.0);
      break;
    case S_DROPBASKET:
      set_robot(0.0, 0.0, -basketpos[0].y, -1.0);
      break;
    default:
      set_robot(0.0, 0.0, 0.0, 0.0);
  }
}

/** Try to get a speech command from a hypothesis string
 *  @param hyp
 *    the hypothesis string
 *  @return the command string
 */
static char *get_speech_command(void) {
  const char *commands[] = { "fetch", "return", "stop", NULL };
  char *hyp;
  size_t lastptr;
  int cmdindex;
  int i;
  hyp = speech::listen();
  lastptr = -1;
  for (i = 0; commands[i] != NULL; i++) {
    char *ptr;
    // TODO: resolve last element bug
    ptr = strstr(hyp, commands[i]);
    if ((size_t)ptr - (size_t)hyp > lastptr) {
      lastptr = (size_t)ptr - (size_t)hyp;
      cmdindex = i;
    }
  }
  if (lastptr == -1) {
    return NULL;
  } else {
    return (char *)commands[cmdindex];
  }
}

/** Limit a value between min and max
 *  @param x
 *    the value
 *  @param min
 *    the lower bound
 *  @param max
 *    the upper bound
 *  @return the limited value
 */
static double limitf(double x, double min, double max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  } else {
    return x;
  }
}

/** Set the robot pose in the global space,
 *  where all the values are in the range (-1.0, 1.0)
 *  @param forward
 *    the forward percentage
 *  @param left
 *    the left percentage
 *  @param raise
 *    the raise percentage
 *  @param grab
 *    the grab percentage
 */
static void set_robot(double forward, double left, double raise, double grab) {
  agent_base.y = limitf(forward, -1.0, 1.0);
  agent_base.yaw = limitf(left, -1.0, 1.0);
  agent_arm.pitch = limitf(raise, -1.0, 1.0);
  agent_arm.yaw = limitf(grab, -1.0, 1.0);
}

/** Set the state that the robot is supposed to be looking for
 *  @param detectingstate
 *    either S_BALL or S_BASKET
 */
static void look_for(int detectingstate) {
  if (detectingstate == S_BALL && visual_detect_type != S_BALL) {
    visual_detect_type = S_BALL;
    set_detection(DETECT_BALL);
  } else if (detectingstate = S_BASKET && visual_detect_type != S_BASKET) {
    visual_detect_type = S_BASKET;
    set_detection(DETECT_BASKET);
  }
}

/** Grab all positions in the image
 *  @param n
 *    the mutable integer used to measure message length,
 *    this is 0 if nothing is recv'd
 *  @return a vector of locations
 */
static std::vector<pose3d_t> grab_new_frame_object(int &n) {
  char buf[128];
  std::vector<pose3d_t> locs;
  n = get_position(buf);
  if (n) {
    pose3d_t pt;
    memset(&pt, 0, sizeof(pose3d_t));
    // check to make sure this is the correct thingy
    sscanf(buf, "%lf %lf %lf\n", &pt.x, &pt.y, &pt.z);
    if (!(pt.z == -1 && pt.y == -1 && pt.x == -1)) {
      locs.push_back(pt);
    }
    updated_locs = locs;
  }
  return updated_locs;
}

/** Get the number of balls in the basket
 *  @return the number of balls
 */
static int num_balls_in_basket(void) {
  if (visual_detect_type == S_BALL) {
    std::vector<pose3d_t> locs;
    int n;
    const double in_basket_baseline = 10.0; // TODO: CONFIGURE ME!
    locs = grab_new_frame_object(n);
    if (n) {
      numballs = 0;
      // set baseline
      for (int i = 0; i < locs.size(); i++) {
        numballs += (locs[i].y > in_basket_baseline) ? 1 : 0;
      }
    }
  }
  return numballs;
}

/** Get all possible ball locations
 *  @return the ball locations
 */
static std::vector<pose3d_t> get_filtered_ball_positions(void) {
  if (visual_detect_type == S_BALL) {
    std::vector<pose3d_t> locs;
    std::vector<pose3d_t> flocs;
    int n;
    const double in_basket_baseline = 10.0; // TODO: CONFIGURE ME!
    locs = grab_new_frame_object(n);
    if (n) {
      for (int i = 0; i < locs.size(); i++) {
        if (locs[i].y <= in_basket_baseline) {
          flocs.push_back(locs[i]);
        }
      }
      ballpos = flocs;
    }
  }
  return ballpos;
}

/** Get the closest object in a location list
 *  @param locs
 *    the location list
 *  @return the closest location
 */
static pose3d_t closest_object(std::vector<pose3d_t> locs) {
  return locs[0];
}

/** Get all possible locations of the basket
 *  @return the most likely basket location
 */
static std::vector<pose3d_t> get_filtered_basket_position(void) {
  if (visual_detect_type == S_BASKET) {
    std::vector<pose3d_t> locs;
    std::vector<pose3d_t> flocs;
    int n;
    locs = grab_new_frame_object(n);
    if (n) {
      flocs.push_back(locs[0]);
    }
    basketpos = flocs;
  }
  return basketpos;
}

/** Queries if a ball at a position can be picked up
 *  @param pos
 *    the position of the object
 *  @return true for now
 */
static bool can_pickup(pose3d_t pos) {
  return true; // for now, keep running
}

/** Queries if a ball can be dropped into the basket
 *  @param pos
 *    the likely position of the basket
 *  @return true if close enough to drop, otherwise false
 */
static bool can_drop(pose3d_t pos) {
  pose3d_t *sonar = robot::sense();
  const double collision_baseline = 3.0; // TODO: CONFIGURE ME!
  // dont really need position, just the ultrasonic sensors
  bool no_left_collision = sonar[0].y > collision_baseline;
  bool no_right_collision = sonar[1].y > collision_baseline;
  return (!no_left_collision && !no_right_collision);
}
