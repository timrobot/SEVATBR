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
#define SYNC_NSEC     500000000

static int limit(int value, int min_value, int max_value);
static double enc2cm(int encoder_reading);
static double enc2deg(int encoder_reading);
static double deg2rad(double deg);
static double mag(const arma::vec &v);
static double cos_rule(double A, double B, double C);
static arma::vec leg_fk_solve(int x, int y, const arma::vec &enc);
static arma::vec leg_ik_solve(const arma::vec &curr, const arma::vec &target);

const static double waist_x = 6.4;
const static double waist_y = 27.3;
const static double waist_radius = 3.1;
const static double waist_height = 33.3;
const static double thigh_pivot_length = 6.4;
const static double thigh_pivot_angle = 2.356194490192345;
const static double thigh_length = 27.3;
const static double thigh_actuator_min = 33.1;
const static double thigh_actuator_max = 58.0;
const static double thigh_length_upper = 7.0;
const static double thigh_length_lower = 20.3;
const static double shin_length = 43.1;
const static double shin_actuator_min = 29.5;
const static double shin_actuator_max = 50.0;
const static double shin_length_upper = 33.0;
const static double shin_length_lower = 10.1;

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
      for (i = 0; i < this->num_connected; i++) {
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
  for (i = 0; i < this->num_connected; i++) {
    switch (this->ids[i]) {
      case NW_DEVID:
        break;
      case NE_DEVID:
        // this is a test suite :D
        if (this->outvalues[0] == this->prevvalues[0] &&
            this->outvalues[1] == this->prevvalues[1] &&
            this->outvalues[2] == this->prevvalues[2] &&
            this->outvalues[3] == this->prevvalues[3]) {
          break;
        }
        sprintf(msg, "[%d %d %d %d]\n",
            ((this->outvalues[0] > 0.0) - (this->outvalues[0] < 0.0)),
            ((this->outvalues[1] > 0.0) - (this->outvalues[1] < 0.0)),
            ((this->outvalues[2] > 0.0) - (this->outvalues[2] < 0.0)),
            ((this->outvalues[3] > 0.0) - (this->outvalues[3] < 0.0)));
        this->prevvalues[0] = this->outvalues[0];
        this->prevvalues[1] = this->outvalues[1];
        this->prevvalues[2] = this->outvalues[2];
        this->prevvalues[3] = this->outvalues[3];
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
  int x, y;
  arma::vec enc(3);

  for (i = 0; i < this->num_connected; i++) {
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
        sscanf(msg, "[%d %lf %lf %lf]\n", &id, &enc(0), &enc(1), &enc(2));
        x = -waist_x;
        y = waist_y;
        enc(0) += deg2rad(75.0);
        this->leg_nw = leg_fk_solve(x, y, enc);
        break;
      case NE_DEVID:
        sscanf(msg, "[%d %lf %lf %lf]\n", &id, &enc(0), &enc(1), &enc(2));
        x = waist_x;
        y = waist_y;
        enc(0) += deg2rad(-15.0);
        this->leg_ne = leg_fk_solve(x, y, enc);
        break;
      case SW_DEVID:
        sscanf(msg, "[%d %lf %lf %lf]\n", &id, &enc(0), &enc(1), &enc(2));
        x = -waist_x;
        y = -waist_y;
        enc(0) += deg2rad(165.0);
        this->leg_sw = leg_fk_solve(x, y, enc);
        break;
      case SE_DEVID:
        sscanf(msg, "[%d %lf %lf %lf]\n", &id, &enc(0), &enc(1), &enc(2));
        x = waist_x;
        y = -waist_y;
        enc(0) += deg2rad(255.0);
        this->leg_se = leg_fk_solve(x, y, enc);
        break;
      default:
        break;
    }
  }
}

/** Reset the robot values
 */
void tachikoma::reset(void) {
  this->leg_nw.zeros();
  this->leg_ne.zeros();
  this->leg_sw.zeros();
  this->leg_se.zeros();
  memset(this->outvalues, 0, sizeof(int) * 4);
  memset(this->prevvalues, 0, sizeof(int) * 4);
}

/** Manually write the values for particular legs
 *  @param legid
 *    the id for which leg to set
 *  @param message
 *    the message to send to the leg
 */
void tachikoma::write_manual(int legid, char *message) {
  int i;
  for (i = 0; i < this->num_connected; i++) {
    if (this->ids[i] == legid) {
      serial_write(&this->connections[i], message);
    }
  }
}

/** Manually read the values for particular legs
 *  @param legid
 *    the id for which leg to get from
 *  @return the message, or NULL if there isn't one
 */
char *tachikoma::read_manual(int legid) {
  int i;
  for (i = 0; i < this->num_connected; i++) {
    if (this->ids[i] == legid) {
      return serial_read(&this->connections[i]);
    }
  }
  return NULL;
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
static double enc2cm(int encoder_reading) {
  const int low_reading = 0; // configure
  const int high_reading = 100; // configure
  const double low_distance = 12.0; // configure, inches
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
static double enc2deg(int encoder_reading) {
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

/** Return the magnitude of a vector
 *  @param v
 *    the vector
 *  @return the magnitude
 */
static double mag(const arma::vec &v) {
  int i;
  double sum = 0.0;
  for (i = 0; i < (int)v.n_elem; i++) {
    sum += v(i) * v(i);
  }
  return sqrt(sum);
}

/** Cosine rule for finding an angle
 *  @param A
 *    side1
 *  @param B
 *    side2
 *  @param C
 *    side3
 *  @return angle perpendicular to side3
 */
static double cos_rule_angle(double A, double B, double C) {
  return acos((C * C - A * A - B * B) / (2.0 * A * B));
}

/** Cosine rule for finding a side
 *  @param A
 *    side1
 *  @param B
 *    side2
 *  @param c
 *    angle3
 *  @return side perpendicular to angle3
 */
static double cos_rule_distance(double A, double B, double c) {
  return sqrt(A * A + B * B - 2.0 * A * B * cos(c));
}

/** Solve the xyz coordinate of the leg using forward kinematics
 *  @param x
 *    offset in the x-axis
 *  @param y
 *    offset in the y-axis
 *  @param enc
 *    the encoder states (rotation, thigh_actuator, shin_acuator)
 *  @return the xyz coordinate
 */
static arma::vec leg_fk_solve(int x, int y, const arma::vec &enc) {
  double cosv, sinv;
  double theta, delta;
  double A, B, C;
  arma::mat T(4, 4);
  arma::vec L(4);
  arma::vec temp(3);

  // solve for reference frame 2
  // find the delta corresponding to the bad differential
  A = cos_rule_distance(thigh_pivot_length, thigh_length, thigh_pivot_angle);
  delta = cos_rule_angle(A, thigh_length, thigh_pivot_length);
  // find the theta after removing the delta for the shin transform
  C = enc2cm(enc(2));
  theta = cos_rule_angle(thigh_length_lower, shin_length_upper, C) - delta - M_PI;
  // find the coordinates of the current leg according to the 2nd reference frame
  L(0) = A + cos(theta) * shin_length;
  L(1) = 0.0;
  L(2) = sin(theta) * shin_length;
  L(3) = 1.0;

  // solve for the transformation in reference frame 1
  // find the theta for the thigh transform
  temp(0) = waist_radius;
  temp(1) = 0.0;
  temp(2) = waist_height;
  B = mag(temp);
  C = enc2cm(enc(1));
  theta = -cos_rule_angle(A, B, C); // why negative? goes the other way
  // do transformation
  cosv = cos(theta);
  sinv = sin(theta);
  T(0, 0) = cosv;   T(0, 1) = 0.0;   T(0, 2) = sinv;  T(0, 3) = 0.0;
  T(1, 0) = 0.0;    T(1, 1) = 1.0;   T(1, 2) = 0.0;   T(1, 3) = 0.0;
  T(2, 0) = -sinv;  T(2, 1) = 0.0;   T(2, 2) = cosv;  T(2, 3) = 0.0;
  T(3, 0) = 0.0;    T(3, 1) = 0.0;   T(3, 2) = 0.0;   T(3, 3) = 1.0;
  L = T * L;

  // solve for the transformation in reference frame 0
  cosv = cos(enc2deg(enc(0)));
  sinv = sin(enc2deg(enc(0)));
  T(0, 0) = cosv;  T(0, 1) = -sinv;  T(0, 2) = 0.0;  T(0, 3) = x;
  T(1, 0) = sinv;  T(1, 1) = cosv;   T(1, 2) = 0.0;  T(1, 3) = y;
  T(2, 0) = 0.0;   T(2, 1) = 0.0;    T(2, 2) = 1.0;  T(2, 3) = 0.0;
  T(3, 0) = 0.0;   T(3, 1) = 0.0;    T(3, 2) = 0.0;  T(3, 3) = 1.0;
  L = T * L;

  return L;
}

/** Solve the encoder values of the legs given a target
 *  @param target
 *    the target vector
 *  @return a vector for the target ticks
 */
static arma::vec leg_ik_solve(const arma::vec &curr, const arma::vec &target) {
  double cosv, sinv;
  double theta, delta;
  double A, B, C;
  arma::vec diff;
  arma::vec temp(3);

  diff = target - curr;

  // delete the 
  C = mag(target);
  cosv = cos(M_PI - thigh_pivot_angle);
  sinv = sin(M_PI - thigh_pivot_angle);
  temp(0) = cosv * thigh_pivot_length + thigh_length;
  temp(2) = sinv * thigh_pivot_length;
  A = mag(temp);
  B = shin_length;
  delta = atan2(sinv * thigh_pivot_length, cosv * thigh_pivot_length + thigh_length);
  theta = cos_rule(A, B, C) + delta;
  

  return delta;
}
