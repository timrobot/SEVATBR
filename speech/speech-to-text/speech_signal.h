#ifndef speech_signal_h
#define speech_signal_h

#include "stt.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct speech_signal {
  unsigned char none;
  unsigned char go;
  unsigned char stop;
  unsigned char fetch;
  unsigned char ret;
} speech_signal_t;

int start_speech_signals(void);
void get_signal(speech_signal_t *sigframe);
void stop_speech_signals(void);

#ifdef __cplusplus
}
#endif

#endif
