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
#include <vector>
#include "tachikoma.h"

#define DEV_BAUD      B38400
#define SYNC_NSEC     500000000
#define ENCODER_WAIST 0
#define ENCODER_THIGH 1
#define ENCODER_SHIN  2

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
bool finished(int legid);

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
const double theta_bounds[8] = {
  0.0, 10.0,
  10.0, 20.0,
  20.0, 30.0,
  30.0, 40.0
};

/** Custom class for vectors, based off of the arma::vec
 */
class tvec : public arma::vec {
  public:
    using arma::vec::vec;
    bool operator==(const tvec &other) {
      if (this->n_rows != other.n_rows) {
        return false;
      }
      for (int i = 0; i < (int)other.n_rows; i++) {
        if ((*this)(i) != other(i)) {
          return false;
        }
      }
      return true;
    }
};

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
  if (this->num_possible == 0) {
    this->disconnect();
    return false;
  }
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
  // try to connect to a couple of them (TACHI_NUM_DEV)
  // and identify their ids
  this->connections = new serial_t[TACHI_NUM_DEV];
  memset(this->connections, 0, sizeof(serial_t) * TACHI_NUM_DEV);
  this->ids = new int[TACHI_NUM_DEV];
  for (i = 0, n = 0; n < TACHI_NUM_DEV && i < this->num_possible; i++) {
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
  if (n == 0) {
    this->disconnect();
    return false;
  } else {
    // reset
    this->init_state_space();
    this->reset();
    this->send();
    this->recv();
    return true;
  }
}

/** Disconnect everything
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

/** Return the number of devices that are connected.
 *  @return the number of devices that are connected
 */
int tachikoma::numconnected(void) {
  return this->num_connected;
}

/** Send output to the communication layer.
 *  @note this is where you do all of the inverse kinematics
 */
void tachikoma::send(void) {
  int i;
  char msg[128]; // NOTE: careful of static sizes!
  int legid;

  double forward;
  double backward;
  double turn_left;
  double turn_right;

  // receive the leg values to do forward kinematics
  this->recv();

  // update the movement
  forward = (double)(this->base[0].y > 0);
  backward = (double)(this->base[0].y < 0);
  turn_left = (double)(this->base[0].yaw > 0);
  turn_right = (double)(this->base[0].yaw < 0);
  this->update(forward, backward, turn_left, turn_right);

  for (i = 0; i < this->num_connected; i++) {
    switch (this->ids[i]) {
      case TACHI_NW_DEVID:
      case TACHI_NE_DEVID:
      case TACHI_SW_DEVID:
      case TACHI_SE_DEVID:
        legid = this->getlegid(this->ids[i]);

        if (tvec(this->outval[legid]) == tvec(this->prevval[legid])) {
          break;
        }
        sprintf(msg, "[%.5lf %.5lf %.5lf]\n",
            this->outval[legid](0),
            this->outval[legid](1),
            this->outval[legid](2));
        this->prevval[legid] = this->outval[legid];
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
  arma::vec encoder(3);

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
        legid = this->getlegid(id);
        sscanf(msg, "[%d %lf %lf %lf]\n", &id,
            &encoder(ENCODER_WAIST),
            &encoder(ENCODER_THIGH),
            &encoder(ENCODER_SHIN));
        this->curr_enc[legid] = encoder;
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
    this->curr_pos[i] = arma::vec(4, arma::fill::zeros);
    this->curr_enc[i] = arma::vec(3, arma::fill::zeros);
    this->target_pos[i] = arma::vec(4, arma::fill::zeros);
    this->target_enc[i] = arma::vec(3, arma::fill::zeros);
    this->outval[i] = arma::vec(3, arma::fill::zeros);
    this->prevval[i] = arma::vec(3, arma::fill::zeros);
  }
  memset(this->base, 0, sizeof(pose3d_t) * 2);
  memset(this->arm, 0, sizeof(pose3d_t) * 2);

  // state space
  this->overall_state = 0;
  this->sub_state = 0;
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

/** Update the states of the robot for the next action
 *  that the robot should do.
 *  @param forward
 *    the speed of the forward (0 to 1)
 *  @param backward
 *    the speed of the backward (0 to 1)
 *  @param turn_left
 *    the speed of the left turning (0 to 1)
 *  @param turn_right
 *    the speed of the right turning (0 to 1)
 */
void tachikoma::update(double forward, double backward, double turn_left, double turn_right) {
  enum m_states { M_STATIC, M_RESET };
  enum p_states { P_TROT_LEFT, P_TROT_RIGHT };
  enum v_states { V_IDLE, V_FORWARD, V_BACKWARD, V_LEFT, V_RIGHT };
  arma::vec default_pos[4];

  // ... lets pretend we made all of them
  int i;
  unsigned long dt;
  bool onchangestate;

  // change the state machine if necessary (careful of these states -
  // they use heuristics which are not machine learned)
  onchangestate = false;
  switch (this->overall_state) {
    case V_IDLE: // idle state
      // place a state here for idle reset
      break;
    case V_FORWARD:
      if (!forward) {
        this->overall_state = V_IDLE;
        onchangestate = true;
      } else {
        switch (this->sub_state) {
          case P_TROT_LEFT:
            if (finished(this->getlegid(TACHI_NW_DEVID)) &&
                finished(this->getlegid(TACHI_SE_DEVID))) {
              this->sub_state = P_TROT_RIGHT;
              onchangestate = true;
            }
            break;
          case P_TROT_RIGHT:
            if (finished(this->getlegid(TACHI_NE_DEVID))) {
              this->sub_state = P_TROT_LEFT;
              onchangestate = true;
            }
            break;
        }
      }
      break;
  }

  if (onchangestate) {
    // save new start time, save current states
  }

  // update the velocity of the mechanism
  switch (this->overall_state) {
    case V_IDLE:
      // TODO
      break;
    case V_FORWARD:
      switch (this->sub_state) {
        case P_TROT_LEFT:
          // find destination vector based on particle filters
          // apply the ik solver after this
          break;
      }
  }

  for (int i = 0; i < 4; i++) {
    this->leg_ik_solve(i, this->target_pos[i]);
    this->outval[i] = this->target_enc[i];
  }
}

/** Lookup the legid of a particular devid.
 *  @param devid
 *    the devid
 *  @return the legid
 */
int tachikoma::getlegid(int devid) {
  return devid - 1;
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
  arma::vec L;

  // define reference frame 3
  L = { shin_length, 0.0, 0.0, 1.0 };

  // solve for the transformation in refrence frame 2
  // find the theta for the shin transform
  A = cos_rule_distance(thigh_pivot_length, thigh_length, thigh_pivot_angle);
  delta = cos_rule_angle(A, thigh_length, thigh_pivot_length); // account for curve
  C = enc2cm(this->curr_enc[legid](ENCODER_SHIN));
  theta = -(cos_rule_angle(thigh_length_lower, shin_length_upper, C) - delta - M_PI);
  // do transformation
  cosv = cos(theta);
  sinv = sin(theta);
  T = reshape(arma::mat({
        cosv,  0.0, sinv, A,
        0.0,   1.0, 0.0,  0.0,
        -sinv, 0.0, cosv, 0.0,
        0.0,   0.0, 0.0,  1.0
        }), 4, 4).t();
  L = T * L;

  // solve for the transformation in reference frame 1
  // find the theta for the thigh transform
  B = cos_rule_distance(waist_radius, waist_height, M_PI_2);
  C = enc2cm(this->curr_enc[legid](ENCODER_THIGH));
  theta = -cos_rule_angle(A, B, C);
  // do transformation
  cosv = cos(theta);
  sinv = sin(theta);
  T = reshape(arma::mat({
        cosv,  0.0, sinv, 0.0,
        0.0,   1.0, 0.0,  0.0,
        -sinv, 0.0, cosv, 0.0,
        0.0,   0.0, 0.0,  1.0
        }), 4, 4).t();
  L = T * L;

  // solve for the transformation in reference frame 0
  theta = this->curr_enc[legid](ENCODER_WAIST) + waist_angle[legid];
  // do transformation
  cosv = cos(theta);
  sinv = sin(theta);
  T = reshape(arma::mat({
        cosv, -sinv, 0.0, waist_x[legid],
        sinv, cosv,  0.0, waist_y[legid],
        0.0,  0.0,   1.0, 0.0,
        0.0,  0.0,   0.0, 1.0
        }), 4, 4).t();
  L = T * L;

  // store the value inside of the leg
  this->curr_pos[legid] = L;
}

/** Solve the encoder values of the legs given a target
 *  @param legid
 *    the id of the leg to do ik on
 *  @param target
 *    the target vector (x, y, z)
 */
void tachikoma::leg_ik_solve(int legid, const arma::vec &target) {
  double theta, delta;
  double A, B, C;
  arma::mat T(4, 4);
  arma::vec L;
  arma::vec encoder(3);

  // invert the transformation for reference frame 0
  L = { target(0) - waist_x[legid], target(1) - waist_y[legid], target(2), 1.0 };

  // invert the transformation for reference frame 1
  encoder(ENCODER_WAIST) = rad2enc(atan2(L(1), L(0)) + waist_angle[legid]);
  L(0) = mag(arma::vec({ L(0), L(1), 0.0 }));
  L(1) = 0.0;

  // invert the transformation for reference frame 3
  A = cos_rule_distance(thigh_pivot_length, thigh_length, thigh_pivot_angle);
  delta = cos_rule_angle(A, thigh_length, thigh_pivot_length); // account for curve
  B = shin_length;
  C = mag(arma::vec({ L(0), L(1), L(2) }));
  theta = cos_rule_angle(A, B, C) + delta;
  encoder(ENCODER_SHIN) = cm2enc(cos_rule_distance(
        thigh_length_lower, shin_length_upper, theta));

  // invert the transformation for reference frame 2
  theta -= delta;
  theta = M_PI_2 + M_PI_4 - theta - atan2(waist_height, waist_radius);
  B = mag(arma::vec({ waist_radius, 0.0, waist_height }));
  encoder(ENCODER_THIGH) = cm2enc(cos_rule_distance(A, B, theta));

  this->target_enc[legid] = encoder;
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
  const double low_distance = 12.0; // configure, cm
  const double high_distance = 34.0; // configure, cm
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
  return sqrt(arma::dot(v, v));
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

// TODO: replace
bool finished(int legid) {
  return true;
}

/** Leg motion vector: lift up and move forward
 */
arma::vec forward_lift(const arma::vec &start, const arma::vec &stop, double t) {
  return arma::vec(4);
}
