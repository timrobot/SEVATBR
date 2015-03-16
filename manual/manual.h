#ifndef manual_h
#define manual_h

#include "coord.h"
#define MNL_SRVR  0x0001
#define MNL_CTRL  0x0002

#ifdef __cplusplus
extern "C" {
#endif

int manual_connect(int type);
void manual_enable(void);
void manual_disable(void);
int manual_disconnect(void);
int manual_new_data(void);
void manual_get_poses(pose3d_t *base, pose3d_t *arm);

#ifdef __cplusplus
}
#endif

#endif
