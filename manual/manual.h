#ifndef manual_h
#define manual_h

#include "iplink.h"
#include "robot.h"

typedef struct manual {
 iplink_t connection;
 robotctrl_t ctrl;
} manual_t;

int manual_connect(manual_t *mnl);
robotctrl_t *manual_get(manual_t *mnl);
int manual_disconnect(manual_t *mnl);

#endif
