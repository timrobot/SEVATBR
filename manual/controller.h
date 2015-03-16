#ifndef controller_h
#define controller_h

#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Controller structure */
typedef struct controller {
  char    *name;      /* name of the device */
  int32_t fd;         /* file descriptor of the device */
  int8_t  connected;  /* is the device connected */
  int32_t buttons;
  int32_t axes;

  /* threaded update */
  uint64_t  thread;
  int8_t    alive;  /* 1 if alive, 0 if dead */

  /* values */
  int8_t  A;
  int8_t  B;
  int8_t  X;
  int8_t  Y;
  int8_t  UP;
  int8_t  DOWN;
  int8_t  LEFT;
  int8_t  RIGHT;
  int8_t  LB;
  int8_t  RB;
  float   LT;
  float   RT;
  struct {
    int8_t  pressed;
    float   x;
    float   y;
  } LJOY, RJOY;
  int8_t  START;
  int8_t  SELECT;
  int8_t  HOME;
} controller_t;

/* Prototypes */
void controller_connect(controller_t *ctrl);
void controller_disconnect(controller_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif
