// VISUAL
#include "tball.h"
#include "basket.h"
// SPEECH
#include "stt.h"
#include "tts.h"
// ROBOT
#include "robot.h"
// AI AGENT
#include "agent.h"

using namespace agent;

/** Wake the agent up, start all initial processes
 *  @return 0 on success, -1 on error
 */
int wakeup(void) {
  // start the vision engine(s)
  // start the speech to text listener
  stt_start_listening();
}

/** Make the agent go to sleep, stop all processes
 */
void sleep(void) {
  // stop the vision engine(s)
  // stop the speech to text listener
  stt_stop_listening();
}

/** Enable the AI
 */
void enable(void) {
  // COMPLETE THIS METHOD
}

/** Disable the AI
 */
void disable(void) {
}

/** Get the poses
 *  @param base
 *    the base struct
 *  @param arm
 *    the arm struct
 *  @return 0
 */
int get_poses(pose3d_t *base, pose3d_t *arm) {
  // COMPLETE THIS METHOD
  return 0;
}
