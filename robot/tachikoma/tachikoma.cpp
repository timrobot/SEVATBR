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
static int cm2enc(double length);
static double enc2rad(int encoder_reading);
static int rad2enc(double radians);
static double deg2rad(double deg);
static double rad2deg(double rad);
static double mag(const arma::vec &v);
static double cos_rule_angle(double A, double B, double C);
static double cos_rule_distance(double A, double B, double c);

const static double waist_x[4] = { -6.4, 6.4, -6.4, 6.4 };
const static double waist_y[4] = { 27.3, 27.3, -27.3, -27.3 };
const static double waist_angle[4] = { 75.0, -15.0, 165.0, -105.0 };
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
    if (id == TACHI_NW_DEVID ||
        id == TACHI_NE_DEVID ||
        id == TACHI_SW_DEVID ||
        id == TACHI_SE_DEVID) {
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
  char msg[128]; // NOTE: careful of static sizes!
  int legid;

  const int bounded_value = 10.0; // 10 degrees

  // receive the leg values to do forward kinematics
  this->recv();
  for (i = 0; i < this->num_connected; i++) {
    switch (this->ids[i]) {
      case TACHI_NW_DEVID:
      case TACHI_NE_DEVID:
      case TACHI_SW_DEVID:
      case TACHI_SE_DEVID:
        legid = this->ids[i] - 1; // NOTE: hacky!

        // look at the values which the robot needs to move,
        // and calculate the trajectory from limits
        // TODO: incorporate arm movement

        // we have already gotten the values for the IK
        // from the recv() call we made earlier
        
      
        if (this->outval[0] == this->prevval[0] &&
            this->outval[1] == this->prevval[1] &&
            this->outval[2] == this->prevval[2] &&
            this->outval[3] == this->prevval[3]) {
          break;
        }
        sprintf(msg, "[%d %d %d %d]\n",
            ((this->outval[0] > 0.0) - (this->outval[0] < 0.0)),
            ((this->outval[1] > 0.0) - (this->outval[1] < 0.0)),
            ((this->outval[2] > 0.0) - (this->outval[2] < 0.0)),
            ((this->outval[3] > 0.0) - (this->outval[3] < 0.0)));
        this->prevval[0] = this->outval[0];
        this->prevval[1] = this->outval[1];
        this->prevval[2] = this->outval[2];
        this->prevval[3] = this->outval[3];
        serial_write(&this->connections[i], msg);
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
  int legid;
  arma::vec enc(3);

  for (i = 0; i < this->num_connected; i++) {
    // read message, and unless valid id or no message, go to computation
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
      case TACHI_NW_DEVID:
      case TACHI_NE_DEVID:
      case TACHI_SW_DEVID:
      case TACHI_SE_DEVID:
        legid = id - 1; // NOTE: hacky!
        sscanf(msg, "[%d %lf %lf %lf]\n", &id, &enc(0), &enc(1), &enc(2));
        this->encoder[legid] = enc;
        this->leg_fk_solve(legid);
        break;
      default:
        break;
    }
  }
}

/** Reset the robot values
 */
void tachikoma::reset(void) {
  int i;
  for (i = 0; i < 4; i++) {
    this->leg[i].zeros();
    this->outval[i].zeros();
    this->prevval[i].zeros();
  }
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

/** Solve the xyz coordinate of the leg using forward kinematics
 *  @param legid
 *    the legid to solve for
 */
void tachikoma::leg_fk_solve(int legid) {
  double cosv, sinv;
  double theta, delta;
  double A, B, C;
  arma::mat T(4, 4);
  arma::vec L(4);
  arma::vec temp(3);

  // TODO: create a standard multiplicative matrix

  // define reference frame 3
  L(0) = shin_length;
  L(1) = 0.0;
  L(2) = 0.0;
  L(3) = 1.0;

  // solve for the transformation in refrence frame 2
  // find the theta for the shin transform
  A = cos_rule_distance(thigh_pivot_length, thigh_length, thigh_pivot_angle);
  delta = cos_rule_angle(A, thigh_length, thigh_pivot_length); // account for curve
  C = enc2cm(this->encoder[legid](2));
  theta = -(cos_rule_angle(thigh_length_lower, shin_length_upper, C) - delta - M_PI);
  // do transformation
  cosv = cos(theta);
  sinv = sin(theta);
  T(0, 0) = cosv;   T(0, 1) = 0.0;  T(0, 2) = sinv;  T(0, 3) = A;
  T(1, 0) = 0.0;    T(1, 1) = 1.0;  T(1, 2) = 0.0;   T(1, 3) = 0.0;
  T(2, 0) = -sinv;  T(2, 1) = 0.0;  T(2, 2) = cosv;  T(2, 3) = 0.0;
  T(3, 0) = 0.0;    T(3, 1) = 0.0;  T(3, 2) = 0.0;   T(3, 3) = 1.0;
  L = T * L;

  // solve for the transformation in reference frame 1
  // find the theta for the thigh transform
  B = cos_rule_distance(waist_radius, waist_height, M_PI_2);
  C = enc2cm(this->encoder[legid](1));
  theta = -cos_rule_angle(A, B, C);
  // do transformation
  cosv = cos(theta);
  sinv = sin(theta);
  T(0, 0) = cosv;   T(0, 1) = 0.0;  T(0, 2) = sinv;  T(0, 3) = 0.0;
  T(1, 0) = 0.0;    T(1, 1) = 1.0;  T(1, 2) = 0.0;   T(1, 3) = 0.0;
  T(2, 0) = -sinv;  T(2, 1) = 0.0;  T(2, 2) = cosv;  T(2, 3) = 0.0;
  T(3, 0) = 0.0;    T(3, 1) = 0.0;  T(3, 2) = 0.0;   T(3, 3) = 1.0;
  L = T * L;

  // solve for the transformation in reference frame 0
  theta = this->encoder[legid](0) + waist_angle[legid];
  // do transformation
  cosv = cos(theta);
  sinv = sin(theta);
  T(0, 0) = cosv;  T(0, 1) = -sinv;  T(0, 2) = 0.0;  T(0, 3) = waist_x[legid];
  T(1, 0) = sinv;  T(1, 1) = cosv;   T(1, 2) = 0.0;  T(1, 3) = waist_y[legid];
  T(2, 0) = 0.0;   T(2, 1) = 0.0;    T(2, 2) = 1.0;  T(2, 3) = 0.0;
  T(3, 0) = 0.0;   T(3, 1) = 0.0;    T(3, 2) = 0.0;  T(3, 3) = 1.0;
  L = T * L;

  // store the value inside of the leg
  this->leg[legid] = L;
}

/** Solve the encoder values of the legs given a target
 *  @param legid
 *    the id of the leg to do ik on
 *  @param target
 *    the target vector (x, y, z)
 *  @return a vector for the target ticks
 */
void tachikoma::leg_ik_solve(int legid, const arma::vec &target) {
  double theta, delta;
  double A, B, C;
  arma::mat T(4, 4);
  arma::vec L(4);
  arma::vec temp(3);
  arma::vec enc(3);

  // TODO: check for size 4 for the target

  // invert the transformation for reference frame 0
  L(0) = target(0) - waist_x[legid];
  L(1) = target(1) - waist_y[legid];
  L(2) = target(2);
  L(3) = 1.0;

  // invert the transformation for reference frame 1
  enc(0) = rad2enc(atan2(L(1), L(0)) + waist_angle[legid]);
  temp(0) = L(0); temp(1) = L(1); temp(2) = 0.0;
  A = mag(temp);
  L(0) = A;
  L(1) = 0.0;

  // invert the transformation for reference frame 3
  A = cos_rule_distance(thigh_pivot_length, thigh_length, thigh_pivot_angle);
  delta = cos_rule_angle(A, thigh_length, thigh_pivot_length); // account for curve
  B = shin_length;
  temp(0) = L(0); temp(1) = L(1); temp(2) = L(2);
  C = mag(temp);
  theta = cos_rule_angle(A, B, C) + delta;
  enc(2) = cm2enc(cos_rule_distance(thigh_length_lower, shin_length_upper, theta));

  // invert the transformation for reference frame 2
  theta = M_PI_2 + M_PI_4 - theta;
  temp(0) = waist_radius; temp(1) = 0.0; temp(2) = waist_height;
  B = mag(temp);
  enc(1) = cm2enc(cos_rule_distance(A, B, theta));

  return enc;
}

/** PRIVATE FUNCTIONS **/

/** Limit an a value between a range
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

/** Conversion from encoder reading to actuator reading
 *  @param encoder_reading
 *    the encoder reading
 *  @return the conversion into distance (centimeters)
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

/** Conversion from encoder reading to actuator reading
 *  @param length
 *    the length in centimeters
 *  @return the conversion into encoder values
 */
static int cm2enc(double length) {
  const int low_reading = 0; // configure
  const int high_reading = 100; // configure
  const double low_distance = 12.0; // configure, inches
  const double high_distance = 34.0; // configure, inches
  double ratio;
  ratio = (high_reading - low_reading) / (double)(high_distance - low_distance);
  // bound values
  length = limit(length, low_distance, high_distance);
  return (int)(ratio * (length - low_distance) + low_reading);
}

/** Conversion from encoder reading into radians
 *  @param encoder_reading
 *    the encoder reading
 *  @return the conversion into radians
 */
static double enc2rad(int encoder_reading) {
  const int low_reading = 0;
  const int high_reading = 100;
  const double low_degree = 0.0;
  const double high_degree = 90.0;
  double ratio;
  ratio = (high_degree - low_degree) / (double)(high_reading - low_reading);
  // boumd values
  encoder_reading = limit(encoder_reading, low_reading, high_reading);
  return deg2rad(ratio * (encoder_reading - low_reading) + low_degree);
}

/** Conversion from radians into encoder reading
 *  @param radians
 *    the radians to input
 *  @return the conversion into encoder readings
 */
static int rad2enc(double radians) {
  const int low_reading = 0;
  const int high_reading = 100;
  const double low_degree = 0.0;
  const double high_degree = 90.0;
  double ratio;
  ratio = (high_reading - low_reading) / (double)(high_degree - low_degree);
  // boumd values
  radians = limit(radians, low_degree, high_degree);
  return (int)deg2rad(ratio * (radians - low_degree) + low_reading);
}

/** Conversion from degrees to radians
 *  @param deg
 *    the degrees
 *  @return radians
 */
static double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

/** Conversion from radians to degrees
 *  @param rad
 *    the radians
 *  @return degrees
 */
static double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
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
