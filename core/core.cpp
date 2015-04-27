/************************************
 * The purpose of this object is to
 * translate different formats of
 * communication to their respective
 * devices. You can think of it as
 * the front end, as well as the
 * starting program.
 *
 * The core backbone relies on the
 * following paradigms:
 * 1) The user will always want to
 *    either control the robot
 *    initially, or transfer
 *    control to the AI until the
 *    wants control back.
 * 2) The robot can be a flexible
 *    machine, with many different
 *    parts or mechanisms, and that
 *    the programmer will want to
 *    connect them simply yet
 *    powerfully.
 ***********************************/
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include "robot.h"
#include "user.h"
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
  pose3d_t base[2];
  pose3d_t arm[2];

  printf("CORE INITIALIZING...\n");
  signal(SIGINT, stop_program);
  // init robot and user
  if (robot::set(TACHIKOMA) == -1) {
    return -1;
  }
  if (user_connect(USER_XBOXCTRL) == -1) {
    return -1;
  }

  // start getting communication accesses
  while (!stop_signal) {
    user_get_poses(base, arm);
    robot::move(base, arm);
  }

  // clean up
  user_disconnect();
  robot::unset();
  printf("CORE COMPLETE.\n");

  return 0;
}
