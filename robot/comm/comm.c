/****************************************
 *                                     
 * The purpose of this program is to do 
 * the following:                       
 *                                      
 * 1) Open and store all the arduinos   
 *    in linked list or hashtable, then 
 *    map each arduino to an id.
 * Sensors:
 * 2) Extract data from each id given
 *    the following format:
 *      [id data_1 data_2 ... data_n]
 * 3) Send those data to a user when they
 *    ask for it through get_<sensor>().
 * Motor:
 * 4) Provide a way for users to issue
 *    direct control through
 *    set_<motor>()
 * 5) Provide a way for users to issue
 *    abstracted control through:
 *    forward()
 *    backward()
 *    turn_left()
 *    turn_right()
 *    stop()
 *    grab()
 *    release()
 *    open_claw()
 *    close_claw()
 * 6) Send those commands to the arduino
 *    through the following format:
 *      [data_1 data_2 ... data_n]\n
 * Miscellaneous:
 * 7) Provide a list of properties:
 *    can_turn_yaw    (on the y-axis)
 *                    (left/right)
 *    can_turn_pitch  (on the x-axis)
 *    can_turn_roll   (on the z-axis)
 *    can_move_y      (forward/backward)
 *    can_move_x      (strafe left/right)
 *    can_move_z      (up/down)
 *    num_arms
 *    num_joints_arm
 *    has_claw
 *    has_conveyor
 *    has_custom
 *    function names
 *
 ***************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <dirent.h>
#include "serial.h"
#include "comm.h"

#define NUM_DEV     2
#define WHEEL_DEVID 1
#define ARM_DEVID   2

static int initialized;
static serial_t connections[NUM_DEV];
static int connection_ids[NUM_DEV];
static char **possible_connections;
static int num_possible;

// data pieces
static int baseleft;
static int baseright;
static int armbottom;
static int armtop;
static int clawrotate;
static int clawleft;
static int clawright;

int comm_init(void) {
  // find all the arduino devices in the device directory
  DIR *device_dir;
  struct dirent *entry;
  int i, n;
  if (initialized) {
    return -1;
  } else {
    initialized = 1;
  }
  device_dir = opendir("/dev/"); // device directory
  // iterate through all the possible filenames in the directory, get count
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      num_possible++;
    }
  }
  closedir(device_dir);
  possible_connections = (char **)malloc(sizeof(char *) * num_possible);
  i = 0;
  device_dir = opendir("/dev/");
  // add all the possible filenames to the list
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      possible_connections[i++] = entry->d_name;
    }
  }
  closedir(device_dir);
  // when finished adding all the possible filenames,
  // try to connect to a couple of them (NUM_DEV)
  // and identify their ids
  for (i = 0, n = 0; n < NUM_DEV && i < num_possible; i++) {
    char *msg;
    int id;
    // connect device
    serial_connect(&connections[n], possible_connections[i], 57600);
    if (!connections[n].connected) {
      continue;
    }
    // read a message
    do  {
      msg = serial_read(&connections[n]);
    } while (strlen(msg) == 0);
    // if a valid device, add as connected, otherwise disconnect
    sscanf(msg, "[%d", &id);
    if (id == WHEEL_DEVID || id == ARM_DEVID) {
      connection_ids[n] = id;
      n++;
    } else {
      serial_disconnect(&connections[n]);
    }
  }
  return n;
}

void comm_update(void) {
  // there is actually nothing that we need at the moment
  int i;
  for (i = 0; i < NUM_DEV; i++) {
    switch (connection_ids[i]) {
      case WHEEL_DEVID:
        break;
      case ARM_DEVID:
        break;
      default:
        break;
    }
  }
}

void comm_flush(void) {
  int i;
  for (i = 0; i < NUM_DEV; i++) {
    char msg[128];
    switch (connection_ids[i]) {
      case WHEEL_DEVID:
        sprintf(msg, "%d %d %d\n", baseleft, baseright, armbottom);
        break;
      case ARM_DEVID:
        sprintf(msg, "%d %d %d %d\n", armtop, clawrotate, clawleft, clawright);
        break;
      default:
        break;
    }
    serial_write(&connections[i], msg);
  }
}

void comm_destroy(void) {
  int i;
  if (initialized) {
    for (i = 0; i < NUM_DEV; i++) {
      serial_disconnect(&connections[i]);
    }
    if (possible_connections) {
      free(possible_connections);
      possible_connections = NULL;
    }
    num_possible = 0;
    memset(connections, 0, sizeof(serial_t) * NUM_DEV);
    memset(connection_ids, 0, sizeof(int) * NUM_DEV);
    initialized = 0;
  }
}
