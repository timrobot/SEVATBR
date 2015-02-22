#ifndef tbr_h
#define tbr_h

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tbr {
  serial_t *connections;
  int *ids;
  int8_t connected;

  int baseleft;
  int baseright;
  int armbottom;
  int armtop;
  int clawrotate;
  int clawleft;
  int clawright;

  char **possible_ports;
  int num_possible;
} tbr_t;

int tbr_connect(tbr_t *robot);
void tbr_send(tbr_t *robot);
void tbr_recv(tbr_t *robot);
void tbr_disconnect(tbr_t *robot);

void tbr_move_forward(tbr_t *robot);
void tbr_move_backward(tbr_t *robot);
void tbr_turn_left(tbr_t *robot);
void tbr_turn_right(tbr_t *robot);
void tbr_stop_wheels(tbr_t *robot);

void tbr_grab_ball(tbr_t *robot);
void tbr_release_ball(tbr_t *robot);
void tbr_open_claw(tbr_t *robot);
void tbr_close_claw(tbr_t *robot);
void tbr_lift_arm(tbr_t *robot);
void tbr_drop_arm(tbr_t *robot);
void tbr_stop_arm(tbr_t *robot);

#ifdef __cplusplus
}
#endif

#endif
