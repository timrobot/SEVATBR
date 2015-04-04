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
#include "tachikoma.h"

#define NUM_DEV       4
#define DEV_BAUD      B38400
#define NW_DEVID      1
#define NE_DEVID      2
#define SW_DEVID      3
#define SE_DEVID      4
#define SYNC_NSEC     500000000

static int limit(int value, int min_value, int max_value);
static double encoder2actuator(int encoder_reading);
static double encoder2deg(int encoder_reading);
static double deg2rad(double deg);
static arma::vec *gen_leg_vec(int top_encoder, int bottom_encoder);
static void leg_ik_solve(void **lp);

/** CLASS FUNCTIONS **/

/** Constructor
 */
tachikoma::tachikoma(void) {
  this->connect();
}

/** Deconstructor
 */
tachikoma::~tachikoma(void) {
  this->disconnect();
}

/** Initialize the communication layer.
 *  @return whether or not the robot was able to connect with a device
 */
bool tachikoma::connect(void) {
  // find all the arduino devices in the device directory
  DIR *device_dir;
  struct dirent *entry;
  int i, n;
  device_dir = opendir("/dev/"); // device directory
  // iterate through all the possible filenames in the directory, get count
  this->num_possible = 0;
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      this->num_possible++;
    }
  }
  closedir(device_dir);
  this->possible_ports = new char *[this->num_possible];
  device_dir = opendir("/dev/");
  i = 0;
  // add all the possible filenames to the list
  while ((entry = readdir(device_dir))) {
    if (strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0 &&
        strstr(entry->d_name, "ttyACM")) {
      char *pport;
      pport = new char[strlen("/dev/") + strlen(entry->d_name) + 1];
      sprintf(pport, "/dev/%s", entry->d_name);
      this->possible_ports[i++] = pport;
    }
  }
  closedir(device_dir);
  // when finished adding all the possible filenames,
  // try to connect to a couple of them (NUM_DEV)
  // and identify their ids
  this->connections = new serial_t[NUM_DEV];
  memset(this->connections, 0, sizeof(serial_t) * NUM_DEV);
  this->ids = new int[NUM_DEV];
  for (i = 0, n = 0; n < NUM_DEV && i < this->num_possible; i++) {
    char *msg;
    int id;
    struct timespec synctime;
    synctime.tv_nsec = SYNC_NSEC % 1000000000;
    synctime.tv_sec = SYNC_NSEC / 1000000000;
    // connect device
    serial_connect(&this->connections[n], this->possible_ports[i], DEV_BAUD);
    if (!this->connections[n].connected) {
      continue;
    }
    // read a message
    nanosleep(&synctime, NULL);
    do  {
      msg = serial_read(&this->connections[n]);
    } while (!msg || strlen(msg) == 0);
    // read another one in case that one was garbage
    nanosleep(&synctime, NULL);
    do {
      msg = serial_read(&this->connections[n]);
    } while (!msg || strlen(msg) == 0);
    // if a valid device, add as connected, otherwise disconnect
    sscanf(msg, "[%d", &id);
    if (id == NW_DEVID || id == NE_DEVID || id == SW_DEVID || id == SE_DEVID) {
      this->ids[n++] = id;
    } else {
      serial_disconnect(&this->connections[n]);
    }
  }

  this->num_connected = n;
  // debug
  printf("Number of devices connected: %d\n", n);
  if (n == 0) {
    this->disconnect();
    return false;
  } else {
    // reset
    this->reset();
    this->send();
    this->recv();
    return true;
  }
}

/** Determine whether or not the robot is connected.
 *  @return true if the robot is connected to a device, else false
 */
bool tachikoma::connected(void) {
  return this->num_connected > 0;
}

/** Send output to the communication layer.
 *  @note this is where you do all of the inverse kinematics
 */
void tachikoma::send(void) {
  int i;
  char msg[128]; // careful of static sizes!
  this->recv(); // just because we need to refresh constantly
  for (i = 0; i < num_connected; i++) {
    switch (this->ids[i]) {
      case NW_DEVID:
        break;
      case NE_DEVID:
        // this is a test suite :D
        if (!this->arm || !this->base ||
            (this->arm->x == this->x &&
             this->arm->y == this->y &&
             this->arm->z == this->z &&
             this->base->x == this->w)) {
          break;
        }
        sprintf(msg, "[%d %d %d %d]\n",
            ((this->arm->x > 0.0) - (this->arm->x < 0.0)),
            ((this->arm->y > 0.0) - (this->arm->y < 0.0)),
            ((this->arm->z > 0.0) - (this->arm->z < 0.0)),
            ((this->base->x > 0.0) - (this->base->x < 0.0))
            );
        this->x = this->arm->x;
        this->y = this->arm->y;
        this->z = this->arm->z;
        this->w = this->base->x;
        serial_write(&this->connections[i], msg);
        break;
      case SW_DEVID:
        break;
      case SE_DEVID:
        break;
      default:
        break;
    }
  }
}

/** Receive input from the communication layer.
 *  @note this is where you do forward kinematics
 */
void tachikoma::recv(void) {
  // there is actually nothing that we need at the moment
  int i;
  char *msg;

  int id;
  int rotate_encoder;
  int top_encoder;
  int bottom_encoder;
  double rotation;
  arma::mat T(4, 4); // transformation matrix
  arma::vec *leg2;
  arma::vec leg1;
  double sinrot;
  double cosrot;

  for (i = 0; i < num_connected; i++) {
    // read message, and unless valid id or no message, goto computation
    if (this->ids[i] != 0) {
      if (!(msg = serial_read(&this->connections[i]))) {
        continue;
      }
    } else {
      continue;
    }
    sscanf(msg, "[%d ", &id);
    this->ids[i] = id;

    // do something with the message (such as compute the forward knematics)
    switch (id) {
      case NW_DEVID:
      case NE_DEVID:
      case SW_DEVID:
      case SE_DEVID:
        sscanf(msg, "[%d %d %d %d]\n", &id, &rotate_encoder, &bottom_encoder, &top_encoder);
        T.zeros();

        // determine the rotation
        rotation = deg2rad(encoder2deg(rotate_encoder));
        switch (id) {
          case NW_DEVID:
            rotation += deg2rad(75.0);
            T(0, 3) = -8.00;
            T(1, 3) = 12.00;
            break;
          case NE_DEVID:
            rotation += deg2rad(-15.0);
            T(0, 3) = 8.00;
            T(1, 3) = 12.00;
            break;
          case SW_DEVID:
            rotation += deg2rad(165.0);
            T(0, 3) = -8.00;
            T(1, 3) = -12.00;
            break;
          case SE_DEVID:
            rotation += deg2rad(255.0);
            T(0, 3) = 8.00;
            T(1, 3) = -12.00;
            break;
        }

        // create a leg vector
        leg2 = gen_leg_vec(top_encoder, bottom_encoder);
        // create a new T matrix
        sinrot = sin(rotation);
        cosrot = cos(rotation);
        T(0, 0) = cosrot;
        T(0, 2) = sinrot;
        T(2, 0) = -sinrot;
        T(2, 2) = cosrot;
        T(1, 1) = 1.0;
        T(3, 3) = 1.0;

        // get the new leg pose, assign to actual leg
        leg1 = T * (*leg2);
        switch (id) {
          case NW_DEVID:
            this->leg_nw = leg1;
            break;
          case NE_DEVID:
            this->leg_ne = leg1;
            break;
          case SW_DEVID:
            this->leg_sw = leg1;
            break;
          case SE_DEVID:
            this->leg_se = leg1;
            break;
        }

        break;
      default:
        break;
    }
  }
}

/** Reset the robot values
 *  @param robot
 *    the robot information
 */
void tachikoma::reset(void) {
  this->leg_nw.zeros();
  this->leg_ne.zeros();
  this->leg_sw.zeros();
  this->leg_se.zeros();
  this->arm = NULL;
  this->base = NULL;
  this->x = 0;
  this->y = 0;
  this->z = 0;
  this->w = 0;
}

/** Disconnect everything
 *  @param robot
 *    the robot information
 */
void tachikoma::disconnect(void) {
  int i;
  if (this->connected()) {
    // reset the robot
    this->reset();
    this->send();
    // delete the connections
    if (this->connections) {
      for (i = 0; i < num_connected; i++) {
        serial_disconnect(&this->connections[i]);
      }
      delete this->connections;
      this->connections = NULL;
    }
    // delete the port names
    if (this->possible_ports) {
      for (i = 0; i < this->num_possible; i++) {
        if (this->possible_ports[i]) {
          delete this->possible_ports[i];
        }
      }
      delete this->possible_ports;
      this->possible_ports = NULL;
    }
    // delete the ids
    if (this->ids) {
      delete this->ids;
      this->ids = NULL;
    }
    this->num_possible = 0;
    this->num_connected = 0;
  }
}

/** PRIVATE FUNCTIONS **/

/** Limit an a value between a range.
 *  @param value
 *    the value to be limited
 *  @param min_value
 *    minimum value
 *  @param max_value
 *    maximum value
 *  @return the limited value
 */
static int limit(int value, int min_value, int max_value) {
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
static double encoder2actuator(int encoder_reading) {
  const int low_reading = 0; // configure
  const int high_reading = 100; // configure
  const double low_distance = 20.0; // configure, inches
  const double high_distance = 34.0; // configure, inches
  double ratio;
  ratio = (high_distance - low_distance) / (double)(high_reading - low_reading);
  // bound values
  encoder_reading = limit(encoder_reading, low_reading, high_reading);
  return ratio * (encoder_reading - low_reading) + low_distance;
}

/** Conversion from encoder reading into degrees.
 *  @param encoder_reading
 *    the encoder reading
 *  @return the conversion into degrees
 */
static double encoder2deg(int encoder_reading) {
  const int low_reading = 0;
  const int high_reading = 100;
  const double low_degree = 0.0;
  const double high_degree = 90.0;
  double ratio;
  ratio = (high_degree - low_degree) / (double)(high_reading - low_reading);
  // boumd values
  encoder_reading = limit(encoder_reading, low_reading, high_reading);
  return ratio * (encoder_reading - low_reading) + low_degree;
}

/** Conversion from degrees to radians.
 *  @param deg
 *    the degrees
 *  @return radians
 */
static double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

/** Forward kinematics for a single leg using the encoder values.
 *  @param final_pose
 *    the pose at which the bottom of the leg is positioned
 *  @param top_encoder
 *    the reading of the top encoder
 *  @param btm_encoder
 *    the reading of the bottom encoder
 */
static arma::vec *gen_leg_vec(int top_encoder, int bottom_encoder) {
  arma::vec *final_pose;
  double bottom_leg_theta;
  double top_leg_theta;
  double bottom_actuator;
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

  final_pose = new arma::vec(4);
  bottom_actuator = encoder2actuator(bottom_encoder);
  top_actuator = encoder2actuator(top_encoder);

  // use law of cosines
  C = bottom_actuator * bottom_actuator;
  A = bottom_lower_length * bottom_lower_length;
  B = bottom_higher_length * bottom_higher_length;
  bottom_leg_theta = acos((C - (A + B)) / (-2 * bottom_lower_length * bottom_higher_length)) - M_PI;

  C = top_actuator * top_actuator;
  A = top_higher_length * top_higher_length;
  B = top_lower_length * top_lower_length;
  top_leg_theta = M_PI_2 - acos((C - (A + B)) / (-2 * top_higher_length * top_lower_length));

  // create the leg vector and send back
  (*final_pose)(0) = top_length * cos(top_leg_theta) +
    bottom_length * cos(bottom_leg_theta + top_leg_theta);
  (*final_pose)(1) = 0.0;
  (*final_pose)(2) = top_length * sin(top_leg_theta) +
    bottom_length * sin(bottom_leg_theta + top_leg_theta);
  (*final_pose)(3) = 1.0;

  return final_pose;
}

static double mag(arma::vec *v) {
  int i;
  double sum = 0.0;
  for (i = 0; i < v->n_elem; i++) {
    sum += (*v)(i) * (*v)(i);
  }
  return sqrt(sum);
}

static double square(double x) {
  return x * x;
}

static arma::vec *leg_ik_solve(arma::vec *target) {
  // define constants
  const double thigh_pivot_length = 10.16; // centimeters
  const double thigh_length = 45.72;
  const double thigh_actuator_min = 0.0;
  const double thigh_actuator_max = 180.0;
  const double thigh_length_upper = 10.16;
  const double thigh_length_lower = 35.56;
  const double shin_length = 76.2;
  const double shin_actuator_min = 0.0;
  const double shin_actuator_max = 180.0;
  const double shin_length_upper = 54.5;
  const double shin_length_lower = 21.7;
  
  // refer to notes
  int i;
  double distance = mag(target);
  double theta = acos((square(distance) - square(thigh_length) - square(shin_length)) /
      (2 * thigh_length * shin_length));
  double alpha = atan2((*target)(0), (*target)(1));
  double beta = acos((square(shin_length) - square(thigh_length) - square(distance)) /
      (2 * thigh_length * distance));
  double phi = M_PI - alpha - beta;


}
