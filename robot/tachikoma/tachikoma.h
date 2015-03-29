#ifndef tachikoma_h
#define tachikoma_h

#include <stdint.h>
#include "serial.h"

#ifdef __cplusplus
extern "C" {
#endif

enum tachileg_t { nwleg = 1, neleg, swleg, seleg };

typedef struct tachikoma {
  serial_t *connections;
  int *ids;
  int8_t connected;

  char **possible_ports;
  int num_possible;

  pose3d_t leg[4];  
} tachikoma_t;

int tachikoma_connect(tachikoma_t *robot);
void tachikoma_send(tachikoma_t *robot);
void tachikoma_recv(tachikoma_t *robot);
void tachikoma_disconnect(tachikoma_t *robot);
void tachikoma_reset(tachikoma_t *robot);

#ifdef __cplusplus
}
#endif

#endif
