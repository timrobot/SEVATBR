#ifndef user_h
#define user_h

#include "xboxctrl.h"
#include "coord.h"
#define USER_SERVER   0x0001
#define USER_XBOXCTRL 0x0002
#define USER_ENABLE   1
#define USER_DISABLE  0

#ifdef __cplusplus
extern "C" {
#endif

  int user_connect(int type);
  int user_disconnect(void);
  void user_set_enable(int en);
  int user_get_override(void);
  int user_get_poses(pose3d_t *base, pose3d_t *arm);
  void user_log(const char *msg);
  xboxctrl_t *user_get_ctrl(void);

#ifdef __cplusplus
}
#endif

#endif
