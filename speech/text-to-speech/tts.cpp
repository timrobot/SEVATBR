#include <festival.h>
#include "tts.h"

static int festival_initialized;
static int voice_selected;
static void tts_init(void);

static void tts_init(void) {
  if (!festival_initialized) {
    festival_initialize(1, 2100000);
    festival_initialized = 1;
  }
}

int tts_select_voice(char const *voicename) {
  char voice_selection[64];
  int res;
  if (!festival_initialized) {
    tts_init();
  }
  sprintf(voice_selection, "(%s)", voicename);
  res = festival_eval_command((EST_String)voice_selection);
  voice_selected = 1;
  return res;
}


int tts_say(char const *msg) {
  if (!voice_selected) {
    tts_select_voice("(voice_kal_diphone)");
  }
  festival_say_text(msg);
  festival_wait_for_spooler();
  return 0;
}
