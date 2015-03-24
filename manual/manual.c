#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <stdlib.h>
#include "httplink.h"
#include "controller.h"
#include "manual.h"

#define HZ  10

static int input_id;
static httplink_t server;
static controller_t ctrl;
static pose3d_t base;
static pose3d_t arm;
static int new_join;
static struct timeval last_signal;
static int manual_en;
static int mnl_override;
static void server_update(void);
static void raise_server_request(int signum);
static void controller_update(void);

// TODO set throttle management for multiple connections
//      as of now, it can only handle one request at a time
//      look at js async request handling for inspiration

/** Connect to the manual connection - do not enable just yet.
 *  @param id
 *    the id of the manual connection to connect to
 *  @return 0 on success, -1 otherwise
 */
int manual_connect(int id) {
  input_id = id;
  switch (id) {
    case MNL_SRVR:
      {
        int res;
        res = httplink_connect(&server, "sevatbr-v002.appspot.com");
        if (res != -1) {
          // assign unthrottle to sigalrm
          struct sigaction action;
          memset(&action, 0, sizeof(struct sigaction));
          action.sa_handler = raise_server_request;
          sigaction(SIGALRM, &action, NULL);
          mnl_override = 1;
        }
        return res;
      }

    case MNL_CTRL:
      controller_connect(&ctrl);
      return 0;
    
    default:
      break;
  }
  return -1;
}

/** Enable manual mode
 */
void manual_enable(void) {
  manual_en = 1;
  if (input_id == MNL_SRVR) {
    struct itimerval timer;
    // enable the timer to raise every 1/HZ time
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 1E6 / HZ;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 1E6 / HZ;
    setitimer(ITIMER_REAL, &timer, NULL);
  }
}

/** Disable manual mode
 */
void manual_disable(void) {
  memset(&base, 0, sizeof(pose3d_t));
  memset(&arm, 0, sizeof(pose3d_t));
  manual_en = 0;
}

/** Disconnect from the manual connection
 *  @return 0, else -1 on error
 */  
int manual_disconnect(void) {
  manual_disable();
  switch (input_id) {
    case MNL_SRVR:
      // kill throttle timer
      return httplink_disconnect(&server);

    case MNL_CTRL:
      controller_disconnect(&ctrl);
      return 0;

    default:
      break;
  }
  return -1;
}

/** Get the status of the join
 *  @return 1 if a new join exists, else 0
 */
int manual_new_data(void) {
  switch (input_id) {
    case MNL_SRVR:
      server_update();
      return new_join;

    case MNL_CTRL:
      return 1;

    default:
      break;
  }
  return 0;
}

/** User override
 *  @return 1 if overridden, else 0
 */
int isOverriden(void) {
  switch (input_id) {
    case MNL_SRVR:
      server_update();
      return mnl_override;

    case MNL_CTRL:
      return 1;
    
    default:
      break;
  }
  return 0;
}


/** Get the poses
 *  @param the structs needed to hold the poses
 */
void manual_get_poses(pose3d_t *b, pose3d_t *a) {
  switch (input_id) {
    case MNL_SRVR:
      server_update(); // flush the server
      new_join = 0;
      memcpy(b, &base, sizeof(pose3d_t));
      memcpy(a, &arm, sizeof(pose3d_t));
      break;

    case MNL_CTRL:
      controller_update();
      memcpy(b, &base, sizeof(pose3d_t));
      memcpy(a, &arm, sizeof(pose3d_t));
      break;

    default:
      memset(b, 0, sizeof(pose3d_t));
      memset(a, 0, sizeof(pose3d_t));
  }
}

/** Private method to get the information sent over from the server
 *  and set the robot with this information
 */
static void server_update(void) {
  char *msg;
  char *sp, *ep;
  char buf[16];
  size_t buflen;
  int ctrlsig;
  int up, down, left, right;
  int lift, drop, grab, release;
  int ovr;

  // get message
  if (!(msg = httplink_recv(&server))) {
    long diff;
    struct timeval currtime;
    // reset after some time
    gettimeofday(&currtime, NULL);
    diff = (currtime.tv_usec - last_signal.tv_usec) +
        (currtime.tv_sec - last_signal.tv_sec) * 1E6;
    if (diff >= 1E6) { // specified time is one second (lost internet connection)
      memset(&base, 0, sizeof(pose3d_t));
      memset(&arm, 0, sizeof(pose3d_t));
      gettimeofday(&last_signal, NULL);
      new_join = 1;
    }
    return;
  }
  sp = strstr(msg, "feedback: ") + sizeof(char) * strlen("feedback: ");
  ep = strstr(sp, " ");
  buflen = (size_t)ep - (size_t)sp;
  if (buflen > 15) {
    // buffer overflow
    return;
  }
  strncpy(buf, sp, buflen);
  buf[buflen] = '\0';
  ctrlsig = atoi(buf);

  // set the signals
  up =      (ctrlsig & 0x00000001) >> 0;
  down =    (ctrlsig & 0x00000002) >> 1;
  left =    (ctrlsig & 0x00000004) >> 2;
  right =   (ctrlsig & 0x00000008) >> 3;
  lift =    (ctrlsig & 0x00000010) >> 4;
  drop =    (ctrlsig & 0x00000020) >> 5;
  grab =    (ctrlsig & 0x00000040) >> 6;
  release = (ctrlsig & 0x00000080) >> 7;
  ovr =     (ctrlsig & 0x00000100) >> 8;

  mnl_override = ovr;

  memset(&base, 0, sizeof(pose3d_t));
  memset(&arm, 0, sizeof(pose3d_t));
  base.y = (double)(up - down);
  base.yaw = (double)(left - right);
  arm.pitch = (double)(lift - drop);
  arm.yaw = (double)(grab - release);

  // update the time and signal
  gettimeofday(&last_signal, NULL);
  new_join = 1;
}

/** Private method to handle server requesting
 *  @param signum
 *    the id for the signal
 */
static void raise_server_request(int signum) {
  if (!manual_en) {
    struct itimerval timer;
    memset(&timer, 0, sizeof(struct itimerval));
    setitimer(ITIMER_REAL, &timer, NULL);
  } else {
    httplink_send(&server, "/manual_feedback", "get", NULL);
  }
}

/** Private to set the base and arm using information
 *  from the controller
 */
void controller_update(void) {
  int ltrunc, rtrunc;
  ltrunc = (ctrl.LJOY.y > 0.4) ? 1 :
      ((ctrl.LJOY.y < -0.4) ? -1 : 0);
  rtrunc = (ctrl.RJOY.x > 0.4) ? 1 :
      ((ctrl.RJOY.x < -0.4) ? -1 : 0);

  memset(&base, 0, sizeof(pose3d_t));
  memset(&arm, 0, sizeof(pose3d_t));
  if (ltrunc != 0) {
    base.y = ltrunc * 1.0;
  } else {
    base.yaw = rtrunc * -1.0;
  }

  arm.pitch = (ctrl.B - ctrl.A) * 1.0;
  arm.yaw = (ctrl.RB - ctrl.LB) * 1.0;
}
