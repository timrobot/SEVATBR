#ifndef speech_h
#define speech_h

namespace speech {
  int start(void);
  void stop(void);
  char *listen(void);
  void say(const char *fmt);
}

#endif
