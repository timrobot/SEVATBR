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
#include "tbr.h"

#define NUM_DEV       2
#define DEV_BAUD      38400
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
  robot->connections = (serial_t *)malloc(sizeof(serial_t) * NUM_DEV);
  robot->ids = (int *)malloc(sizeof(int) * NUM_DEV);
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
    tbr_reset(robot);
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
        sprintf(msg, "%d %d\n",
            robot->arm,
            robot->claw);
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
    switch (robot->ids[i]) {
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
    tbr_reset(robot);
    tbr_send(robot);
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

/** Reset the robot values
 *  @param robot
 *    the robot information
 */
void tbr_reset(tbr_t *robot) {
  robot->baseleft = 0;
  robot->baseright = 0;
  robot->arm = 0;
  robot->claw = 0;
}
