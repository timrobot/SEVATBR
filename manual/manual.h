#ifndef manual_h
#define manual_h

#include "coord.h"

#ifdef __cplusplus
extern "C" {
#endif

int manual_connect(void);
void manual_enable(void);
void manual_disable(void);
int manual_disconnect(void);
int manual_new_data(void);
void manual_get_poses(pose3d_t *base, pose3d_t *arm);

#ifdef __cplusplus
}
#endif

#endif
