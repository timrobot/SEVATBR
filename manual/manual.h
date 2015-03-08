#ifndef manual_h
#define manual_h

#include "iplink.h"
#include "coord.h"

typedef struct manual {
 iplink_t connection;
 pose3d_t ctrl;
 int enable;
} manual_t;

int manual_connect(manual_t *mnl);
int *manual_set_robot(robot_t *robot);
int manual_disconnect(manual_t *mnl);

#endif
