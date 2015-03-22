/************************************
 * The purpose of this object is to
 * translate different formats of
 * communication to their respective
 * devices. You can think of it as
 * the front end, as well as the
 * starting program.
 ***********************************/
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include "robot.h"
#include "manual.h"
#include "agent.h"
#include "coord.h"

// Signal handler for killing the program
static int stop_signal;
void stop_program(int signum) {
  stop_signal = 1;
}

/** This is the starting program for the robot
 */
int main(int argc, char *argv[]) {
  signal(SIGINT, stop_program);
  // init robot and manual
  if (robot_set(TENNIS_BALL_ROBOT) == -1) {
    return -1;
  }
  //manual_connect(MNL_CTRL);
  agent_create(AGENT_SIMPLE);

  // change later
  //manual_enable();
  agent_enable();

  // start getting communication accesses
  while (!stop_signal) {
    pose3d_t base;
    pose3d_t arm;
    // choose input
    //if (isOverriden()) {
      //manual_get_poses(&base, &arm);
    //} else {
      agent_enable();
      agent_get_poses(&base, &arm);
    //}
    robot_move(&base, &arm);
  }

  // clean up
  agent_destroy();
  //manual_disconnect();
  robot_unset();

  return 0;
}
