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
#include <math.h>
#include "coord.h"
#include "matrix.h"
#include "tachikoma.h"

#define NUM_DEV       4
#define DEV_BAUD      B38400
#define NW_DEVID      1
#define NE_DEVID      2
#define SW_DEVID      3
#define SE_DEVID      4
#define SYNC_NSEC     500000000

/** Initialize the communication layer
 *  @param robot
 *    the robot information
 *  @return number of devices initialized, -1 on error
 */
int tachikoma_connect(tachikoma_t *robot) {
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
      char *pport;
      pport = (char *)malloc(sizeof(char) * (strlen("/dev/") + strlen(entry->d_name) + 1));
      sprintf(pport, "/dev/%s", entry->d_name);
      robot->possible_ports[i++] = pport;
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
    // if a valid device, add as connected, otherwise disconnect
    sscanf(msg, "[%d", &id);
    if (id == NW_DEVID || id == NE_DEVID || id == SW_DEVID || id == SE_DEVID) {
      robot->ids[n++] = id;
    } else {
      serial_disconnect(&robot->connections[n]);
    }
  }

  robot->connected = 1;
  // debug
  printf("Number of devices connected: %d\n", n);
  // disconnect if number of devices is not enough, or there are too many
  if (n != NUM_DEV) {
    tachikoma_disconnect(robot);
    return -1;
  } else {
    // reset
    tachikoma_reset(robot);
    tachikoma_send(robot);
    tachikoma_recv(robot);
    return n;
  }
}

/** Limit an a value between a range.
 *  @param value
 *    the value to be limited
 *  @param min_value
 *    minimum value
 *  @param max_value
 *    maximum value
 *  @return the limited value
 */
int limit(int value, int min_value, int max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}

/** Conversion from encoder reading to actuator reading.
 *  @param encoder_reading
 *    the encoder reading
 *  @return the conversion into distance (inches)
 */
double cvt_encoder_actuator(int encoder_reading) {
  const int low_reading = 0; // configure
  const int high_reading = 100; // configure
  const double low_distance = 20.0; // configure, inches
  const double high_distance = 34.0; // configure, inches
  double ratio;
  ratio = (high_distance - low_distance) / (double)(high_reading - low_reading);
  // bound values
  encoder_reading = limit(encoder_reading, low_reading, high_reading);
  return ratio * (encoder_reading - low_reading) + low_reading;
}

/** Forward kinematics for a single leg using the encoder values.
 *  @param final_pose
 *    the pose at which the bottom of the leg is positioned
 *  @param top_encoder
 *    the reading of the top encoder
 *  @param btm_encoder
 *    the reading of the bottom encoder
 */
void leg_fk_solve(pose3d_t *final_pose, int top_encoder, int btm_encoder) {
  double bottomleg_theta;
  double topleg_theta;
  double btm_actuator;
  double top_actuator;
  
  // configure the following values
  // note: cannot be zero!
  const double bottom_lower_length = 18.0;
  const double bottom_higher_length = 10.0;
  const double top_lower_length = 10.0;
  const double top_higher_length = 18.0;
  const double bottom_length = 24.00;
  const double top_length = 18.0;

  double C;
  double A;
  double B;

  btm_actuator = cvt_encoder_actuator(btm_encoder);
  top_actuator = cvt_encoder_actuator(top_encoder);
  
  // use law of cosines
  C = btm_actuator * btm_actuator;
  A = bottom_lower_length * bottom_lower_length;
  B = bottom_higher_length * bottom_higher_length;
  bottomleg_theta = acos((C - (A + B)) / (-2 * bottom_lower_length * bottom_higher_length));
  
  C = top_actuator * top_actuator;
  A = top_higher_length * top_higher_length;
  B = top_lower_length * top_lower_length;
  topleg_theta = acos((C - (A + B)) / (-2 * top_higher_length * top_lower_length));

  // improper fk implemenetation, but works faster
  memset(final_pose, 0, sizeof(pose3d_t));
  final_pose->x = top_length * sin(topleg_theta) -
      bottom_length * cos(bottomleg_theta + M_PI_2 - topleg_theta);
  final_pose->y = top_length * cos(topleg_theta) -
      bottom_length * sin(bottomleg_theta + M_PI_2 - topleg_theta);
}

/** Send output to the communication layer.
 *  This is where you do all of the inverse kinematics.
 *  @param robot
 *    the robot information
 */
void tachikoma_send(tachikoma_t *robot) {
  int i;
  tachikoma_recv(robot);
  for (i = 0; i < NUM_DEV; i++) {
    char msg[128]; // careful of static sizes!
    switch (robot->ids[i]) {
      case NW_DEVID:
        serial_write(&robot->connections[i], msg);
        break;
      case NE_DEVID:
        serial_write(&robot->connections[i], msg);
        break;
      case SW_DEVID:
        serial_write(&robot->connections[i], msg);
        break;
      case SE_DEVID:
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
void tachikoma_recv(tachikoma_t *robot) {
  // there is actually nothing that we need at the moment
  int i;
  for (i = 0; i < NUM_DEV; i++) {
    switch (robot->ids[i]) {
      default:
        break;
    }
  }
}

/** Disconnect everything
 *  @param robot
 *    the robot information
 */
void tachikoma_disconnect(tachikoma_t *robot) {
  int i;
  if (robot->connected) {
    tachikoma_reset(robot);
    tachikoma_send(robot);
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
void tachikoma_reset(tachikoma_t *robot) {
}
