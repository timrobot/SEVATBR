#ifndef tbr_h
#define tbr_h

#include <stdint.h>
#include "serial.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tbr {
  serial_t *connections;
  int *ids;
  int8_t connected;

  char **possible_ports;
  int num_possible;

  double left;
  double right;
  double arm;
  double claw;

  double prev_left;
  double prev_right;
  double prev_arm;
  double prev_claw;

  double sonar[3];
  int potentiometer;
} tbr_t;

int tbr_connect(tbr_t *robot);
void tbr_send(tbr_t *robot);
void tbr_recv(tbr_t *robot);
void tbr_disconnect(tbr_t *robot);
void tbr_reset(tbr_t *robot);

#ifdef __cplusplus
}
#endif

#endif
