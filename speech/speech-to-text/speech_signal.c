#include "speech_signal.h"
#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

static unsigned char sslib_init;
static unsigned char sslib_exit;
static stt_t sre;
static pthread_t gst_manager;
static speech_signal_t signals;
static void *_update_signals(void *);

int start_speech_signals(void) {
  if (!sslib_init) {
    sslib_init = 1;
    sslib_exit = 0;
    stt_init(&sre);
    if (pthread_create(&gst_manager, NULL, _update_signals, NULL) != 0) {
      stt_free(&sre);
      sslib_init = 0;
      return -1;
    }
    return 0;
  }
  return -1;
}

static void *_update_signals(void *args) {
  // start the process for gstreamer for the signals
  // one process for now, later on add two for consistency and test it out
  int pid;
  char *buf;
  int nread;
  while (!sslib_exit) {
    if ((pid = fork()) == 0) {
      execlp("./readRaw.sh", "readRaw.sh", NULL);
    } else {
      sleep(1); // sleep the process to wait for a command
      kill(pid, SIGINT);
      waitpid(pid, NULL, 0);
      nread = stt_decipher(&sre, "raw_sample.pcm", &buf);
      if (nread) {
        signals.go = strstr(buf, "go") != NULL;
        signals.stop = strstr(buf, "stop") != NULL;
        signals.fetch = strstr(buf, "fetch") != NULL;
        signals.ret = strstr(buf, "return") != NULL;
        signals.none = !(signals.go || signals.stop ||
            signals.fetch || signals.ret);
      }
    }
  }
  pthread_exit(NULL);
  return NULL;
}

void get_signal(speech_signal_t *sigframe) {
  memcpy(sigframe, &signals, sizeof(speech_signal_t));
}

void stop_speech_signals(void) {
  if (sslib_init) {
    sslib_exit = 1;
    pthread_join(gst_manager, NULL);
    sslib_exit = 0;
    stt_free(&sre);
    sslib_init = 0;
  }
}
