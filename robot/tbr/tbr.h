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

  int left;
  int right;
  int arm;
  int claw;

  int prev_left;
  int prev_right;
  int prev_arm;
  int prev_claw;
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
