#ifndef agent_h
#define agent_h

#include "coord.h"
#define AGENT_SIMPLE 0x0001

#ifdef __cplusplus
extern "C" {
#endif

int agent_create(int type);
void agent_enable(void);
void agent_disable(void);
void agent_destroy(void);
void agent_get_poses(pose3d_t *base, pose3d_t *arm);

#ifdef __cplusplus
}
#endif

#endif
