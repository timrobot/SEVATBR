/************************************
 * The purpose of this object is to
 * translate different formats of
 * communication to their respective
 * devices. You can think of it as
 * the front end, as well as the
 * starting program.
 ***********************************/
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include "coord.h"
#include "robot.h"
#include "manual.h"

// Signal handler for killing the program
static int stop_signal;
void stop_program(int signum) {
  stop_signal = 1;
}

/** This is the starting program for the robot
 *  think of it as init.
 */
int main(int argc, char *argv[]) {
  manual_t mnl;
  void *ctrl;
  int manual_mode;

  // set everything
  robot_set(TENNIS_BALL_ROBOT);
  manual_connect(&mnl);
  manual_mode = 1;

  // start getting communication accesses
  while (!stop_signal) {
    // choose input
    if (manual_mode) {
      if ((ctrl = manual_get(&mnl))) {
        robot_set_controls(ctrl);
      }
    } else {
      // sorry dont connect ai right now
    }
  }

  // clean up
  manual_disonnect(&mnl);
  robot_unset();

  return 0;
}
