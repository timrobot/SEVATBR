#ifndef manual_h
#define manual_h

#include "xboxctrl.h"
#include "coord.h"
#define MANUAL_SERVER   0x0001
#define MANUAL_XBOXCTRL 0x0002
#define MANUAL_ENABLE   1
#define MANUAL_DISABLE  0

#ifdef __cplusplus
extern "C" {
#endif

  int manual_connect(int type);
  int manual_disconnect(void);
  void manual_set_enable(int en);
  int manual_get_ovrreq(void);
  int manual_get_poses(pose3d_t *base, pose3d_t *arm);
  xboxctrl_t *manual_get_ctrl(void);

#ifdef __cplusplus
}
#endif

#endif
