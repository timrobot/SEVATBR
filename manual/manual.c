#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include "httplink.h"
#include "manual.h"

#define HZ  10

static httplink_t server;
static pose3d_t base;
static pose3d_t arm;
static int new_join;
static struct timeval last_signal;
static int manual_en;
static void server_update(void);
static void raise_server_request(int signum);

// TODO set throttle management for multiple connections
//      as of now, it can only handle one request at a time
//      look at js async request handling for inspiration

/** Connect to the server. Do not enable just yet.
 *  @return 0 on success, -1 otherwise
 */
int manual_connect(void) {
  int res;
  res = httplink_connect(&server, "sevatbr-v002.appspot.com");
  if (res != -1) {
    // assign unthrottle to sigalrm
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = raise_server_request;
    sigaction(SIGALRM, &action, NULL);
  }
  return res;
}

/** Enable manual mode
 */
void manual_enable(void) {
  struct itimerval timer;
  manual_en = 1;
  // enable the timer to raise every 1/HZ time
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = 1E6 / HZ;
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = 1E6 / HZ;
  setitimer(ITIMER_REAL, &timer, NULL);
}

/** Disable manual mode
 */
void manual_disable(void) {
  memset(&base, 0, sizeof(pose3d_t));
  memset(&arm, 0, sizeof(pose3d_t));
  manual_en = 0;
}

/** Disconnect from the server
 *  @return 0
 */  
int manual_disconnect(void) {
  // kill throttle timer
  manual_disable();
  return httplink_disconnect(&server);
}

/** Get the status of the join
 *  @return 1 if a new join exists, else 0
 */
int manual_new_data(void) {
  server_update();
  return new_join;
}

/** Get the poses
 *  @param the structs needed to hold the poses
 */
void manual_get_poses(pose3d_t *b, pose3d_t *a) {
  server_update();
  memcpy(b, &base, sizeof(pose3d_t));
  memcpy(a, &arm, sizeof(pose3d_t));
  new_join = 0;
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

  // get message
  if (!(msg = httplink_recv(&server))) {
    long diff;
    struct timeval currtime;
    // reset after some time
    gettimeofday(&currtime, NULL);
    diff = (currtime.tv_usec - last_signal.tv_usec) +
        (currtime.tv_sec - last_signal.tv_sec) * 1E6;
    if (diff >= 1E6) { // specified time is one second
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
