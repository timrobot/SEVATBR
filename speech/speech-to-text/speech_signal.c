#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include "rawrec.h"
#include "stt.h"
#include "speech_signal.h"

static unsigned char sslib_init;
static unsigned char sslib_exit;
static stt_t sre;
static pthread_t gst_manager;
static pthread_mutex_t gst_lock;
static speech_signal_t gst_sig;
static void *_update_signals(void *);
#define READPROG "sample_raw.sh"

/** Start recording
*/
int start_speech_signals(void) {
  if (!sslib_init) {
    sslib_init = 1;
    sslib_exit = 0;
    stt_init(&sre);
    pthread_mutex_init(&gst_lock, NULL);
    if (pthread_create(&gst_manager, NULL, _update_signals, NULL) != 0) {
      stt_free(&sre);
      sslib_init = 0;
      return -1;
    }
    return 0;
  }
  return -1;
}

/** Start two streams to record audio.
 *  @param args
 *    Does nothing.
 *  @return NULL
 */
static void *_update_signals(void *args) {
  rawrec_t rec[2];
  char *buf;
  char *fname[2];
  int rid;
  speech_signal_t signals;

  fname[0] = "sample0.raw";
  fname[1] = "sample1.raw";
  start_recording(&rec[0], fname[0]);
  start_recording(&rec[1], fname[1]);
  rid = 0;
  while (!sslib_exit) {
    sleep(1); // sleep the process to wait for a command
    stop_recording(&rec[rid]);
    printf("deciphering %d...\n", rid);
    if ((stt_decipher(&sre, fname[rid], &buf)) > 0) {
      // get the signals
      signals.go = strstr(buf, "go") != NULL;
      signals.stop = strstr(buf, "stop") != NULL;
      signals.fetch = strstr(buf, "fetch") != NULL;
      signals.ret = strstr(buf, "return") != NULL;
      signals.none = !(signals.go || signals.stop ||
          signals.fetch || signals.ret);
      // copy them over
      pthread_mutex_lock(&gst_lock);
      memcpy(&gst_sig, &signals, sizeof(speech_signal_t));
      pthread_mutex_unlock(&gst_lock);
    }
    printf("PS: %s\n", buf);
    unlink(fname[rid]);
    start_recording(&rec[rid], fname[rid]);
    rid = (rid + 1) % 2;
  }
  stop_recording(&rec[0]);
  stop_recording(&rec[1]);
  if (access(fname[0], F_OK)) {
    unlink(fname[0]);
  }
  if (access(fname[1], F_OK)) {
    unlink(fname[1]);
  }
  pthread_exit(NULL);
  return NULL;
}

/** Gets the current state of signals for the sigframe
 *  @param sigframe
 *    the sigframe to send over in order to copy the signals
 */
void get_signal(speech_signal_t *sigframe) {
  pthread_mutex_lock(&gst_lock);
  memcpy(sigframe, &gst_sig, sizeof(speech_signal_t));
  pthread_mutex_unlock(&gst_lock);
}

/** Stop recording
*/
void stop_speech_signals(void) {
  if (sslib_init) {
    sslib_exit = 1;
    pthread_join(gst_manager, NULL);
    sslib_exit = 0;
    pthread_mutex_destroy(&gst_lock);
    stt_free(&sre);
    sslib_init = 0;
  }
}
