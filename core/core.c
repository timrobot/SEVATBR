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
#include "robot.h"
#include "manual.h"
#include "coord.h"

// Signal handler for killing the program
static int stop_signal;
void stop_program(int signum) {
  stop_signal = 1;
}

/** This is the starting program for the robot
 */
int main(int argc, char *argv[]) {
  int manual_mode;

  // init robot and manual
  if (robot_set(TENNIS_BALL_ROBOT) == -1) {
    return -1;
  }
  manual_connect(MNL_SRVR);
  manual_mode = 1;

  // change later
  manual_enable();

  // start getting communication accesses
  while (!stop_signal) {
    // choose input
    if (manual_mode) {
      pose3d_t base;
      pose3d_t arm;
      manual_get_poses(&base, &arm);
      robot_move(&base, &arm);
    }
  }

  // clean up
  manual_disconnect();
  robot_unset();

  return 0;
}
