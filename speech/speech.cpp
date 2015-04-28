#include "stt.h"
#include "tts.h"
#include "speech.h"

static char readbuf[128];

// TODO: DOXY COMMENTS

int speech::start(void) {
  int res;
  tts_select_voice("cmu_us_rms_cg");
  res = stt_start_listening();
  return res;
}

void speech::stop(void) {
  stt_stop_listening();
}

char *speech::listen(void) {
  stt_listen(readbuf);
  return readbuf;
}

void speech::say(const char *fmt) {
  // TODO: account for var arg in
  tts_say(fmt);
}
