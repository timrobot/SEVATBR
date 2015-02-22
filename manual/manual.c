#include <signal.h>
#include "manual.h"

#define HZ  10

// TODO add throttle
static int throttle_en;
static void unthrottle(int signum);

int manual_connect(manual_t *mnl) {
  int res;
  memset(&mnl->ctrl, 0, sizeof(robot_t));
  res = iplink_connect(&mnl->connection, "sevatbr-v002.appspot.com");
  if (res != -1) {
    struct itimerval timer;
    struct sigaction action;
    // assign unthrottle to sigalrm
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = unthrottle;
    sigaction(SIGALRM, &action, NULL);
    // raise alarm every 1 / HZ time
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 1E6 / HZ;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 1E6 / HZ;
    setitimer(ITIMER_REAL, &timer, NULL);
  }
}

robotctrl_t *manual_get(manual_t *mnl) {
  // conform to the specifications in robotctrl_t
  char *sp, *ep;
  char buf[16];
  size_t buflen;
  int ctrlsig;
  int up, down, left, right;

  if (throttle_en) {
    return NULL;
  } else {
    throttle_en = 1;
  }

  // extract message
  sp = strstr(msg, "feedback: ") + sizeof(char) * strlen("feedback: ");
  ep = strstr(sp, " ");
  buflen = (size_t)ep - (size_t)sp;
  if (buflen > 15) {
    /* buffer overflow */
    return NULL;
  }
  strncpy(buf, sp, buflen);
  buf[buflen] = '\0';
  controlsig = atoi(buf);

  // set the signals
  up = ctrlsig & 0x00000001;
  down = (ctrlsig & 0x00000002) >> 1;
  left = (ctrlsig & 0x00000004) >> 2;
  right = (ctrlsig & 0x00000008) >> 3;
  memset(&mnl->ctrl, 0, sizeof(mnl->ctrl));
  mnl->ctrl.y = up - down;
  mnl->ctrl.yaw = right - left;
  return &mnl->ctrl;
}

int manual_disconnect(manual_t *mnl) {
  // kill throttle timer
  struct itimerval timer;
  memset(&timer, 0, sizeof(struct itimerval));
  setitimer(ITIMER_REAL, &timer, NULL);
  return iplink_disconnect(&mnl->connection);
}

static void unthrottle(int signum) {
  throttle_en = 0;
}
