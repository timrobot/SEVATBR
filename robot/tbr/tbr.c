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
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include <time.h>
#include "tbr.h"

#define NUM_DEV       3
#define DEV_BAUD      B38400
#define WHEEL_DEVID   1
#define ARM_DEVID     2
#define CLAW_DEVID    3
#define LEFT_SONAR    0
#define RIGHT_SONAR   1
#define BACK_SONAR    2
#define SYNC_NSEC     500000000

const int botpot = 688;
const int midpot = 650; // TODO: CONFIGURE ME
const int toppot = 630;

static double limitf(double x, double min, double max);

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
  if (robot->num_possible == 0) {
    tbr_disconnect(robot);
    return -1;
  }
  robot->possible_ports = (char **)malloc(sizeof(char *) * robot->num_possible);
  device_dir = opendir("/dev/");
  i = 0;
  // add all the possible filenames to the list
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      char *pport;
      pport = (char *)malloc(sizeof(char) * (strlen("/dev/") + strlen(entry->d_name) + 1));
      sprintf(pport, "/dev/%s", entry->d_name);
      robot->possible_ports[i++] = pport;
      // debug
      printf("possible port: %s\n", pport);
    }
  }
  closedir(device_dir);
  // when finished adding all the possible filenames,
  // try to connect to a couple of them (NUM_DEV)
  // and identify their ids
  robot->connections = (serial_t *)calloc(NUM_DEV, sizeof(serial_t));
  robot->ids = (int *)malloc(sizeof(int) * NUM_DEV);
  for (i = 0, n = 0; n < NUM_DEV && i < robot->num_possible; i++) {
    char *msg;
    int id;
    struct timespec synctime;
    synctime.tv_nsec = SYNC_NSEC % 1000000000;
    synctime.tv_sec = SYNC_NSEC / 1000000000;
    // connect device
    serial_connect(&robot->connections[n], robot->possible_ports[i], DEV_BAUD);
    if (!robot->connections[n].connected) {
      continue;
    }
    // read a message
    nanosleep(&synctime, NULL);
    do  {
      msg = serial_read(&robot->connections[n]);
    } while (!msg || strlen(msg) == 0);
    // read another one in case that one was garbage
    nanosleep(&synctime, NULL);
    do {
      msg = serial_read(&robot->connections[n]);
    } while (!msg || strlen(msg) == 0);
    // debug
    printf("Message: %s\n", msg);
    // if a valid device, add as connected, otherwise disconnect
    sscanf(msg, "[%d ", &id);
    if (id == WHEEL_DEVID || id == ARM_DEVID || id == CLAW_DEVID) {
      robot->ids[n++] = id;
    } else {
      serial_disconnect(&robot->connections[n]);
    }
  }

  robot->connected = 1;
  // debug
  printf("number of devices connected: %d\n", n);
  // disconnect if number of devices is not enough, or there are too many
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
  tbr_recv(robot);
  for (i = 0; i < NUM_DEV; i++) {
    char msg[128]; // careful of static sizes!
    switch (robot->ids[i]) {
      case WHEEL_DEVID:
        if (robot->left == robot->prev_left &&
            robot->right == robot->prev_right) {
          break;
        } else {
          robot->prev_left = robot->left;
          robot->prev_right = robot->right;
        }
        if (robot->left >= 0.0) {
          robot->left *= 1.15;
        }
        if (robot->right >= 0.0) {
          robot->right *= 1.15;
        }
        sprintf(msg, "[%d %d]\n",
            (int)(limitf(robot->left, -1.0, 1.0) * 255.0),
            (int)(limitf(robot->right, -1.0, 1.0) * 255.0));
        serial_write(&robot->connections[i], msg);
        break;
      case ARM_DEVID:
        if (robot->arm == robot->prev_arm) {
          break;
        } else {
          robot->prev_arm = robot->arm;
        }
        // limit the arm
        /*if ((robot->potentiometer > botpot && robot->arm < 0.0) ||
            (robot->potentiometer < toppot && robot->arm > 0.0)) {
          robot->arm = 0.0;
        }*/
        sprintf(msg, "[%d]\n",
            (int)(limitf(robot->arm, -1.0, 1.0) * 255.0));
        serial_write(&robot->connections[i], msg);
        break;
      case CLAW_DEVID:
        if (robot->claw == robot->prev_claw) {
          break;
        } else {
          robot->prev_claw = robot->claw;
        }
        if (robot->claw == 0) {
          sprintf(msg, " ");
        } else if (robot->claw > 0) {
          sprintf(msg, "k");
        } else if (robot->claw < 0) {
          sprintf(msg, "o");
        }
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
  char *msg;
  int i;
  int back_sonar;
  int left_sonar;
  int right_sonar;
  int bt;
  for (i = 0; i < NUM_DEV; i++) {
    switch (robot->ids[i]) {
      case WHEEL_DEVID:
        msg = serial_read(&robot->connections[i]);
        if (!msg) {
          break;
        }
        sscanf(msg, "[%d %d]\n", &robot->ids[i],
            &back_sonar);
        robot->sonar[BACK_SONAR] = (double)back_sonar;
        break;
      case ARM_DEVID:
        msg = serial_read(&robot->connections[i]);
        if (!msg) {
          break;
        }
        sscanf(msg, "[%d %d %d]\n", &robot->ids[i],
            &left_sonar, &right_sonar);
        robot->sonar[LEFT_SONAR] = (double)left_sonar;
        robot->sonar[RIGHT_SONAR] = (double)right_sonar;
        break;
      case CLAW_DEVID:
        msg = serial_read(&robot->connections[i]);
        if (!msg) {
          break;
        }
        sscanf(msg, "[%d %d]\n", &robot->ids[i], &bt);
        break;
      default:
        break;
    }
  }
  // adjust for sonar blockage by the arm
  if (robot->potentiometer < 650) {
    robot->sonar[0] = 200;
    robot->sonar[1] = 200;
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
      for (i = 0; i < robot->num_possible; i++) {
        free(robot->possible_ports[i]);
      }
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
  robot->left = 0;
  robot->right = 0;
  robot->arm = 0;
  robot->claw = 0;
  robot->prev_left = 0;
  robot->prev_right = 0;
  robot->prev_arm = 0;
  robot->prev_claw = 0;
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
