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
#include "measurements.h"

#define DEV_BAUD      B38400
#define SYNC_NSEC     100000000
#define ENCODER_WAIST 0
#define ENCODER_THIGH 1
#define ENCODER_SHIN  2

static int limit(int value, int min_value, int max_value);
static double limitf(double value, double min_value, double max_value);
static double enc2cm(int reading);
static int cm2enc(double length);
static double enc2rad(int reading);
static int rad2enc(double radians);
static double mag(const arma::vec &v);
static double cos_rule_angle(double A, double B, double C);
static double cos_rule_distance(double A, double B, double c);
static arma::vec sin_motion(const arma::vec &start, const arma::vec &stop, double t);
static arma::vec linear_motion(const arma::vec &start, const arma::vec &stop, double t);

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
    sscanf(msg, "[%d ", &id);
    if (id == TACHI_NW_LEG_DEVID ||
        id == TACHI_NE_LEG_DEVID ||
        id == TACHI_SW_LEG_DEVID ||
        id == TACHI_SE_LEG_DEVID ||
        id == TACHI_NW_WHEEL_DEVID ||
        id == TACHI_NE_WHEEL_DEVID ||
        id == TACHI_SW_WHEEL_DEVID ||
        id == TACHI_SE_WHEEL_DEVID) {
      printf("Identified [%s] as [%d]\n", this->possible_ports[i], id);
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
    // this->init_state_space(); // only needed for walking
    this->reset();
    this->update(this->base[0], this->base[1], this->arm[0], this->arm[1]);
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

/** Determine whether or not the robot is connected
 *  @return true if the robot is connected to a device, else false
 */
bool tachikoma::connected(void) {
  return this->num_connected > 0;
}

/** Return the number of devices that are connected
 *  @return the number of devices that are connected
 */
int tachikoma::numconnected(void) {
  return this->num_connected;
}

/** Update the robot's incoming and outgoing signals
 *  @param wheelbase
 *    the pose for the wheels
 *  @param legbase
 *    the pose for the legs and waist
 *  @param leftarm
 *    the pose for the left arm
 *  @param rightarm
 *    the pose for the right arm
 */
int tachikoma::update(pose3d_t wheelbase, pose3d_t legbase,
    pose3d_t leftarm, pose3d_t rightarm) {
//  int i;
  memcpy(&this->base[0], &wheelbase, sizeof(pose3d_t));
  memcpy(&this->base[1], &legbase, sizeof(pose3d_t));
  memcpy(&this->arm[0], &legbase, sizeof(pose3d_t));
  memcpy(&this->arm[1], &legbase, sizeof(pose3d_t));
  this->recv();

  // update the legs
//  for (i = 0; i < TACHI_NUM_LEG_DEV; i++) {
//    this->leg_fk_solve(i);
//  }
  this->update_stand();
  this->update_drive();
  // TODO add conversion from target enc to actual
  this->send();
  return 0;
}

/** Observe the current world
 *  @return NULL for now
 */
pose3d_t *tachikoma::observe(void) {
  return NULL;
}

/** Reset the robot values
 */
void tachikoma::reset(void) {
  int i;
  for (i = 0; i < TACHI_NUM_LEG_DEV; i++) {
    this->curr_pos[i] = arma::vec(4, arma::fill::zeros);
    this->curr_enc[i] = arma::vec(3, arma::fill::zeros);
    this->target_pos[i] = arma::vec(4, arma::fill::zeros);
    this->target_enc[i] = arma::vec(3, arma::fill::zeros);
    memset(this->legval[i], 0, sizeof(int) * 3);
    memset(this->plegval[i], 0, sizeof(int) * 3);
  }
  for (i = 0; i < TACHI_NUM_WHEEL_DEV; i++) {
    memset(this->wheelval[i], 0, sizeof(int));
    memset(this->wheelval[i], 0, sizeof(int));
  }
  memset(this->base, 0, sizeof(this->base) * 2);
  memset(this->arm, 0, sizeof(this->arm) * 2);

  // state space
  this->overall_state = 0;
  this->sub_state = 0;
}

/** Manually write the values for particular legs
 *  @param devid
 *    the id for which device
 *  @param message
 *    the message to send to the leg
 */
void tachikoma::write_manual(int devid, char *message) {
  int i;
  for (i = 0; i < this->num_connected; i++) {
    if (this->ids[i] == devid) {
      serial_write(&this->connections[i], message);
    }
  }
}

/** Manually read the values for particular legs
 *  @param devid
 *    the id for which device
 *  @return the message, or NULL if there isn't one
 */
char *tachikoma::read_manual(int devid) {
  int i;
  for (i = 0; i < this->num_connected; i++) {
    if (this->ids[i] == devid) {
      return serial_read(&this->connections[i]);
    }
  }
  return NULL;
}

/** Lookup the legid of a particular devid.
 *  @param devid
 *    the devid
 *  @return the legid
 */
int tachikoma::getlegid(int devid) {
  return devid - 1;
}

/** Lookup the wheelid of a particular devid.
 *  @param devid
 *    the devid
 *  @return the wheelid
 */
int tachikoma::getwheelid(int devid) {
  return devid - 5;
}

/** Initialize the state space
 */
void tachikoma::init_state_space(void) {
  arma::mat forward_transform = arma::reshape(arma::mat({
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 12.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
        }), 4, 4).t();
  // TODO: configure below
  arma::vec default_leg_pos[4];
  default_leg_pos[1] = arma::vec({ waist_x[1] + 20.0, waist_y[1], -30, 1.0 });
  this->leg_seq[1].add_action(actionstate(
      default_leg_pos[1],
      forward_transform * default_leg_pos[1],
      sin_motion));
  this->leg_seq[1].add_action(actionstate(
      forward_transform * default_leg_pos[1],
      default_leg_pos[1],
      linear_motion));
}

/** Send output to the communication layer
 */
void tachikoma::send(void) {
  int i;
  char msg[128]; // NOTE: careful of static sizes!
  int legid;
  int wheelid;

  for (i = 0; i < this->num_connected; i++) {
    switch (this->ids[i]) {
      case TACHI_NW_LEG_DEVID:
      case TACHI_NE_LEG_DEVID:
      case TACHI_SW_LEG_DEVID:
      case TACHI_SE_LEG_DEVID:
        legid = this->getlegid(this->ids[i]);
        if (this->legval[legid][0] == this->plegval[legid][0] &&
            this->legval[legid][1] == this->plegval[legid][1] &&
            this->legval[legid][2] == this->plegval[legid][2]) {
          break;
        }
        sprintf(msg, "[%d %d %d]\n",
            limit(this->legval[legid][0], -255, 255),
            limit(this->legval[legid][1], -255, 255),
            limit(this->legval[legid][2], -255, 255));
        this->plegval[legid][0] = this->legval[legid][0];
        this->plegval[legid][1] = this->legval[legid][1];
        this->plegval[legid][2] = this->legval[legid][2];
        serial_write(&this->connections[i], msg);
        break;
      case TACHI_NW_WHEEL_DEVID:
      case TACHI_NE_WHEEL_DEVID:
      case TACHI_SW_WHEEL_DEVID:
      case TACHI_SE_WHEEL_DEVID:
        wheelid = this->getwheelid(this->ids[i]);
        if (this->wheelval[wheelid][0] == this->pwheelval[wheelid][0]) {
            break;
        }
        sprintf(msg, "[%d]\n",
            limit(this->wheelval[wheelid][0], -255, 255));
        this->pwheelval[wheelid][0] = this->wheelval[wheelid][0];
        serial_write(&this->connections[i], msg);
        break;
      default:
        break;
    }
  }
}

/** Receive input from the communication layer
 */
void tachikoma::recv(void) {
  int i;
  char *msg;
  int legid;
//  int wheelid;
  int encoder[3];

  for (i = 0; i < this->num_connected; i++) {
    int n;
    // read message, and unless valid id or no message, go to computation
    if (this->ids[i] != 0) {
      if (!(msg = serial_read(&this->connections[i]))) {
        continue;
      }
    } else {
      continue;
    }

    // update the id of the connection
    sscanf(msg, "[%d ", &this->ids[i]);

    switch (this->ids[i]) {
      case TACHI_NW_LEG_DEVID:
      case TACHI_NE_LEG_DEVID:
      case TACHI_SW_LEG_DEVID:
      case TACHI_SE_LEG_DEVID:
        legid = this->getlegid(this->ids[i]);
        sscanf(msg, "[%d %d %d %d]\n", &this->ids[i],
            &encoder[ENCODER_WAIST],
            &encoder[ENCODER_THIGH],
            &encoder[ENCODER_SHIN]);
        for (n = 0; n < 3; n++) {
          this->curr_enc[legid](n) = (double)encoder[n];
        }
        break;
      case TACHI_NW_WHEEL_DEVID:
      case TACHI_NE_WHEEL_DEVID:
      case TACHI_SW_WHEEL_DEVID:
      case TACHI_SE_WHEEL_DEVID:
//        wheelid = this->getwheelid(this->ids[i]);
        break;
      default:
        break;
    }
  }
}

/** Update the walking pose
 *  @param forward
 *    the speed of the forward (0 to 1)
 *  @param backward
 *    the speed of the backward (0 to 1)
 *  @param turn_left
 *    the speed of the left turning (0 to 1)
 *  @param turn_right
 *    the speed of the right turning (0 to 1)
 */
void tachikoma::update_walk(double forward, double backward, double turn_left, double turn_right) {
  // TODO: COMPLETE THIS METHOD
}

/** Update the standing pose
 */
void tachikoma::update_stand(void) {
  const double stand_high_thigh_actuator = 300.0;
  const double stand_high_shin_actuator = 300.0;
  const double stand_mid_thigh_actuator = 200.0;
  const double stand_mid_shin_actuator = 200.0;
  const double stand_low_thigh_actuator = 100.0;
  const double stand_low_shin_actuator = 100.0;
  int i;
  for (i = 0; i < 4; i++) {
    if (this->base[1].z > 0.0) {
      this->target_enc[i](ENCODER_THIGH) = stand_high_thigh_actuator;
      this->target_enc[i](ENCODER_SHIN) = stand_high_shin_actuator;
    } else if (this->base[1].z < 0.0) {
      this->target_enc[i](ENCODER_THIGH) = stand_low_thigh_actuator;
      this->target_enc[i](ENCODER_SHIN) = stand_low_shin_actuator;
    } else {
      this->target_enc[i](ENCODER_THIGH) = stand_mid_thigh_actuator;
      this->target_enc[i](ENCODER_SHIN) = stand_mid_shin_actuator;
    }
    this->legval[i][ENCODER_THIGH] = 0;
    this->legval[i][ENCODER_SHIN] = 0;
  }
}

/** Update the wheels and their poses
 */
void tachikoma::update_drive(void) {
  double angle;
  arma::vec angles(4);
  arma::vec enc(4);
  arma::vec p;
//  arma::vec q[4];
  const int k = 5; // speed factor
  int i;
  arma::vec speed(4);
  arma::vec direction(4);
  arma::vec turning(4);
  arma::vec wheel_vel;
  double vel;
 
  // TODO: test
  // Three particular states:
  // 1) forward motion
  // 2) sideways motion
  // 3) everything else

  if (this->base[0].y != 0.0 &&
      this->base[0].x == 0.0 &&
      this->base[0].yaw == 0.0) {
    angle = 0.0;
    vel = this->base[0].y * 255.0;
    direction = arma::vec({ -1.0, 1.0, -1.0, 1.0 });
    wheel_vel = direction * vel;
    for (i = 0; i < 4; i++) {
      this->target_enc[i](ENCODER_WAIST) = waist_pot_read_min[i] +
          rad2enc(angle - waist_pot_min[i]);
      this->legval[i][0] = k * (int)round(this->target_enc[i](ENCODER_WAIST) -
          this->curr_enc[i](ENCODER_WAIST));
      this->wheelval[i][0] = (int)round(wheel_vel(i) * 255.0);
    }
  } else if (this->base[0].y == 0.0 &&
             this->base[0].x != 0.0 &&
             this->base[0].yaw == 0.0) {
    angle = M_PI_2;
    vel = this->base[0].x * 255.0;
    direction = arma::vec({ -1.0, -1.0, 1.0, 1.0 });
    wheel_vel = direction * vel;
    for (i = 0; i < 4; i++) {
      this->target_enc[i](ENCODER_WAIST) = waist_pot_read_min[i] +
          rad2enc(angle - waist_pot_min[i]);
      this->legval[i][0] = k * (int)round(this->target_enc[i](ENCODER_WAIST) -
          this->curr_enc[i](ENCODER_WAIST));
      this->wheelval[i][0] = (int)round(wheel_vel(i) * 255.0);
    }
  } else {
    // FOR NOW, DO A SIMPLE IMPLEMENTATION, no speed accimation
    angle = M_PI_4; // this will have to be calibrated
//    p = arma::vec({ this->base[0].x, this->base[0].y });
    wheel_vel = arma::vec({
        limitf(-base[0].y - base[0].x + base[0].yaw, -1.0, 1.0),
        limitf( base[0].y - base[0].x + base[0].yaw, -1.0, 1.0),
        limitf(-base[0].y + base[0].x + base[0].yaw, -1.0, 1.0),
        limitf( base[0].y + base[0].x + base[0].yaw, -1.0, 1.0)});
    for (i = 0; i < 4; i++) {
      this->target_enc[i](ENCODER_WAIST) = waist_pot_read_min[i] +
          rad2enc(angle - waist_pot_min[i]);
      this->legval[i][0] = k * (int)round(this->target_enc[i](ENCODER_WAIST) -
          this->curr_enc[i](ENCODER_WAIST));
      this->wheelval[i][0] = (int)round(wheel_vel(i) * 255.0);
    }

    // place the angle onto the robot's waist motors
    if (0) {
      arma::vec q[4];
    for (i = 0; i < TACHI_NUM_WHEEL_DEV; i++) {
      this->target_enc[i](ENCODER_WAIST) = rad2enc(angle);
      // get wheel angles
      angles(i) = waist_pot_min[i] + enc2rad(-waist_pot_read_min[i] +
          this->curr_enc[i](ENCODER_WAIST));
      // move the wheels according the to direction of the current angle, and its normal
      q[i] = arma::vec({ -sin(angles(i)), cos(angles(i)) });
      speed(i) = arma::dot(p, q[i]);
      direction(i) = speed(i) / abs(speed(i));
      speed(i) = abs(speed(i));
      turning(i) = base[0].yaw;
    }
    // normalize according to the largest speed
    speed /= arma::max(speed);
    wheel_vel = speed % direction + turning;
    for (i = 0; i < TACHI_NUM_WHEEL_DEV; i++) {
      this->wheelval[i][0] = round(wheel_vel(i) * 255.0);
    }}
  }
}

/** Solve the xyz coordinate of the leg using forward kinematics
 *  @param legid
 *    the legid to solve for
 */
void tachikoma::leg_fk_solve(int legid) {
  double cosv, sinv;
  double theta, delta;
  double A, B, C;
  arma::mat T;
  arma::vec L;

  // define reference frame 3
  L = { shin_length, 0.0, 0.0, 1.0 };

  // solve for the transformation in refrence frame 2
  // find the theta for the shin transform
  A = cos_rule_distance(thigh_pivot_length, thigh_length, thigh_pivot_angle);
  delta = cos_rule_angle(A, thigh_length, thigh_pivot_length); // account for curve
  C = enc2cm(this->curr_enc[legid](ENCODER_SHIN));
  theta = -(cos_rule_angle(thigh_lower_length, shin_upper_length, C) - delta - M_PI);
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
  B = cos_rule_distance(thigh_x, thigh_z, M_PI_2);
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
        thigh_lower_length, shin_upper_length, theta));

  // invert the transformation for reference frame 2
  theta -= delta;
  theta = M_PI_2 + M_PI_4 - theta - atan2(thigh_z, thigh_x);
  B = mag(arma::vec({ thigh_x, 0.0, thigh_z }));
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

/** Limit an a value between a range (double)
 *  @param value
 *    the value to be limited
 *  @param min_value
 *    minimum value
 *  @param max_value
 *    maximum value
 *  @return the limited value
 */
static double limitf(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  } else if (value > max_value) {
    return max_value;
  } else {
    return value;
  }
}

/** Conversion from encoder reading to actuator reading
 *  @param reading
 *    the encoder reading
 *  @return the conversion into distance (centimeters)
 */
static double enc2cm(int reading) {
  double ratio;
  ratio = (actuator_max - actuator_min) / (double)(actuator_read_max - actuator_read_min);
  reading = limit(reading, actuator_read_min, actuator_read_max);
  return ratio * (reading - actuator_read_min) + actuator_min;
}

/** Conversion from encoder reading to actuator reading
 *  @param length
 *    the length in centimeters
 *  @return the conversion into encoder values
 */
static int cm2enc(double length) {
  double ratio;
  ratio = (double)(actuator_read_max - actuator_read_min) / (actuator_max - actuator_min);
  length = limitf(length, actuator_min, actuator_max);
  return ratio * (length - actuator_min) + actuator_read_min;
}

/** Conversion from encoder reading into radians
 *  @param reading
 *    the encoder reading
 *  @return the conversion into radians
 */
static double enc2rad(int reading) {
  return reading / pot_rad_ratio;
}

/** Conversion from radians into encoder reading
 *  @param radians
 *    the radians to input
 *  @return the conversion into encoder readings
 */
static int rad2enc(double radians) {
  return radians * pot_rad_ratio;
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

/** Leg motion vector: sin function
 *  @param start
 *    the start position
 *  @param stop
 *    the stop position
 *  @param t
 *    the ratio of time (0.0 - 1.0)
 *  @return the approximated curve position
 */
static arma::vec sin_motion(const arma::vec &start, const arma::vec &stop, double t) {
  double theta = M_PI * t;
  arma::vec diff = stop - start;
  double height = 10.0; // TODO : configure
  arma::vec delta = diff * t;
  delta(1) += sin(theta / mag(diff)) * height;
  return start + delta;
}

/** Leg motion vector: linear function
 *  @param start
 *    the start position
 *  @param stop
 *    the stop position
 *  @param t
 *    the ratio of time (0.0 - 1.0)
 *  @return the approximated curve position
 */
static arma::vec linear_motion(const arma::vec &start, const arma::vec &stop, double t) {
  arma::vec diff = stop - start;
  arma::vec delta = diff * t;
  return start + (stop - start) * t;
}
