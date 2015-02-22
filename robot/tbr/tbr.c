/****************************************
 *
 * The purpose of this program is to do 
 * the following for this particular bot:      
 *
 *  1) control the robot through
 *     abstracted methods
 *  2) send back sensor map values
 *
 ***************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <dirent.h>
#include "serial.h"
#include "comm.h"

#define NUM_DEV       2
#define DEV_BAUD      9600
#define WHEEL_DEVID   1
#define ARM_DEVID     2

/** Initialize the communication layer
 *  @param robot
 *    the robot information
 *  @return number of devices initialized, -1 on error
 */
int tbr_connect(tbr_t *robot) {
  // find all the arduino devices in the device directory
  DIR *device_dir;
  struct dirent *entry;
  int i, n;
  device_dir = opendir("/dev/"); // device directory
  // iterate through all the possible filenames in the directory, get count
  robot->num_possible = 0;
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      robot->num_possible++;
    }
  }
  closedir(device_dir);
  robot->possible_ports = (char **)malloc(sizeof(char *) * robot->num_possible);
  device_dir = opendir("/dev/");
  i = 0;
  // add all the possible filenames to the list
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      robot->possible_ports[i++] = entry->d_name;
    }
  }
  closedir(device_dir);
  // when finished adding all the possible filenames,
  // try to connect to a couple of them (NUM_DEV)
  // and identify their ids
  robot->connections = (serial *)malloc(sizeof(serial_t) * NUM_DEV);
  robot->ids = (int *)malloc(sizeof(int_t) * NUM_DEV);
  for (i = 0, n = 0; n < NUM_DEV && i < robot->num_possible; i++) {
    char *msg;
    int id;
    // connect device
    serial_connect(&robot->connections[n], robot->possible_ports[i], DEV_BAUD);
    if (!robot->connections[n].connected) {
      continue;
    }
    // read a message
    do  {
      msg = serial_read(&robot->connections[n]);
    } while (strlen(msg) == 0);
    // if a valid device, add as connected, otherwise disconnect
    sscanf(msg, "[%d", &id);
    if (id == WHEEL_DEVID || id == ARM_DEVID) {
      robot->ids[n] = id;
      n++;
    } else {
      serial_disconnect(&robot->connections[n]);
    }
  }

  robot->connected = 1;
  // disconnect if number of devices is not enough
  if (n != NUM_DEV) {
    tbr_disconnect(robot);
    return -1;
  } else {
    // reset
    robot->baseleft = 0;
    robot->baseright = 0;
    robot->armbottom = 0;
    robot->armtop = 0;
    robot->clawrotate = 0;
    robot->clawleft = 0;
    robot->clawright = 0;
    tbr_send(robot);
    tbr_recv(robot);
    return n;
  }
}

/** Send output to the communication layer
 *  @param robot
 *    the robot information
 */
void tbr_send(tbr_t *robot) {
  int i;
  for (i = 0; i < NUM_DEV; i++) {
    char msg[128]; // careful of static sizes!
    switch (robot->ids[i]) {
      case WHEEL_DEVID:
        int l, r;
        l = robot->baseleft > 0 ? 1 : (robot->baseleft < 0 ? -1 : 0);
        r = robot->baseright > 0 ? 1 : (robot->baseleft < 0 ? -1 : 0);
        if (robot->baseleft == 0 && robot->baseright == 0) {
          sprintf(msg, " ");
        } else if (robot->baseleft > 0 && robot->baseright > 0) {
          sprintf(msg, "w");
        } else if (robot->baseleft < 0 && robot->baseright < 0) {
          sprintf(msg, "s");
        } else if (robot->baseleft > 0 || robot->baseright < 0) {
          sprintf(msg, "d");
        } else if (robot->baseleft < 0 || robot->baseright > 0) {
          sprintf(msg, "a");
        }
        serial_write(&robot->connections[i], msg);
        break;
      case ARM_DEVID:
        sprintf(msg, "%d %d %d %d\n",
            robot->armtop,
            robot->clawrotate,
            robot->clawleft,
            robot->clawright);
        serial_write(&robot->connections[i], msg);
        break;
      default:
        break;
    }
  }
}

/** Receive input from the communication layer
 *  @param robot
 *    the robot information
 */
void tbr_recv(tbr_t *robot) {
  // there is actually nothing that we need at the moment
  int i;
  for (i = 0; i < NUM_DEV; i++) {
    switch (robot->connection_ids[i]) {
      case WHEEL_DEVID:
        break;
      case ARM_DEVID:
        break;
      default:
        break;
    }
  }
}

/** Disconnect everything
 *  @param robot
 *    the robot information
 */
void tbr_disconnect(tbr_t *robot) {
  int i;
  if (robot->connected) {
    if (robot->connections) {
      for (i = 0; i < NUM_DEV; i++) {
        serial_disconnect(&robot->connections[i]);
      }
      free(robot->connections);
      robot->connections = NULL;
    }
    if (robot->possible_ports) {
      free(robot->possible_ports);
      robot->possible_ports = NULL;
    }
    if (robot->ids) {
      free(robot->ids);
      robot->ids = NULL;
    }
    robot->num_possible = 0;
    robot->connected = 0;
  }
}

////////////////////////////////////////////////////////////
// Note: the following functions are abstractions of the  //
// top functions.                                         //
////////////////////////////////////////////////////////////

/** Move the robot forward
 *  @param robot
 *    the robot information
 */
void tbr_move_forward(tbr_t *robot) {
  robot->baseleft = 1;
  robot->baseright = 1;
  tbr_send(robot);
}

/** Move the robot backward
 *  @param robot
 *    the robot information
 */
void tbr_move_backward(tbr_t *robot) {
  robot->baseleft = -1;
  robot->baseright = -1;
  tbr_send(robot);
}

/** Turn the robot left
 *  @param robot
 *    the robot information
 */
void tbr_turn_left(tbr_t *robot) {
  robot->baseleft = -1;
  robot->baseright = 1;
  tbr_send(robot);
}

/** Turn the robot right
 *  @param robot
 *    the robot information
 */
void tbr_turn_right(tbr_t *robot) {
  robot->baseleft = 1;
  robot->baseright = -1;
  tbr_send(robot);
}

/** Stop the wheels from moving
 *  @param robot
 *    the robot information
 */
void tbr_stop_wheels(tbr_t *robot) {
  robot->baseleft = 0;
  robot->baseright = 0;
  tbr_send(robot);
}

/** Create a sequence to grab the ball
 *  @param robot
 *    the robot information
 */
void tbr_grab_ball(tbr_t *robot) {
  // TODO
}

/** Create a sequence to release the ball into a basket
 *  @param robot
 *    the robot information
 */
void tbr_release_ball(tbr_t *robot) {
  // TODO
}

/** Open the claw
 *  @param robot
 *    the robot information
 */
void tbr_open_claw(tbr_t *robot) {
  // TODO
}

/** Close the claw
 *  @param robot
 *    the robot information
 */
void tbr_close_claw(tbr_t *robot) {
  // TODO
}

/** Lift up the arm
 *  @param robot
 *    the robot information
 */
void tbr_lift_arm(tbr_t *robot) {
  // TODO
}

/** Lower the arm
 *  @param robot
 *    the robot information
 */
void tbr_drop_arm(tbr_t *robot) {
  // TODO
}

/** Stop arm movement
 *  @param robot
 *    the robot information
 */
void tbr_stop_arm(tbr_t *robot) {
  // TODO
}
