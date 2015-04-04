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
#include "coord.h"

// Signal handler for killing the program
static int stop_signal;

/** Signal handler to stop the program
 *  @param signum
 *    the signal number (kernel dependent)
 */
void stop_program(int signum) {
  stop_signal = 1;
}

/** This is the starting program for the robot
 *  @param argc
 *    standard exec argument number
 *  @param argv
 *    standard exec argument list
 *  @return 0 on success, -1 otherwise
 */
int main(int argc, char *argv[]) {
  signal(SIGINT, stop_program);
  // init robot and manual
  if (robot_set(TACHIKOMA) == -1) {
    return -1;
  }
  if (manual_connect(MNL_CTRL) == -1) {
    return -1;
  }
  manual_enable();


  // start getting communication accesses
  while (!stop_signal) {
    pose3d_t base;
    pose3d_t arm;
    manual_get_poses(&base, &arm);
    robot_move(&base, &arm);
  }

  // clean up
  manual_disconnect();
  robot_unset();
  printf("done w/ robot\n");

  return 0;
}
