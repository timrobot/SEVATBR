#include "speech_signal.h"
#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>

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
    //stt_init(&sre);
    pthread_mutex_init(&gst_lock, NULL);
    if (pthread_create(&gst_manager, NULL, _update_signals, NULL) != 0) {
      //stt_free(&sre);
      sslib_init = 0;
      return -1;
    }
    return 0;
  }
  return -1;
}

/** Start two processes to record audio.
 *  @param args
 *    Does nothing.
*/
static void *_update_signals(void *args) {
  int pid[2];
  char *buf;
  char *audfile[2];
  int pidindex;

  audfile[0] = "sample1.raw";
  audfile[1] = "sample2.raw";
  pidindex = 0;
  while (!sslib_exit) {
    if ((pid[0] = fork()) == 0) {
      // start the processes for gstreamer for the signals
      if ((pid[1] = fork()) == 0) {
        printf("<<<<<<<<<<<<<<<<<<<<<< Starting first exec! >>>>>>>>>>>>>>>>>>>>>>\n");
        execlp("./" READPROG, READPROG, audfile[1], NULL);
        fprintf(stderr, "failed\n");
      } else {
        printf("<<<<<<<<<<<<<<<<<<<<<< Starting zeroeth exec! >>>>>>>>>>>>>>>>>>>>>>\n");
        execlp("./" READPROG, READPROG, audfile[0], NULL);
        fprintf(stderr, "failed\n");
      }
    } else {
      sleep(1); // sleep the process to wait for a command
      kill(pid[pidindex], SIGINT);
      waitpid(pid[pidindex], NULL, 0);
/*      if ((stt_decipher(&sre, audfile[pidindex], &buf)) > 0) {
        printf("PS: %s\n", buf);
        // get the signals
        speech_signal_t signals;
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
      }*/
      printf("deciphering %d...\n", pidindex);
      if ((pid[pidindex] = fork()) == 0) {
        execlp("./" READPROG, READPROG, audfile[pidindex], NULL);
      }
      pidindex = (pidindex + 1) % 2;
    }
  }
  for (pidindex = 0; pidindex < 2; pidindex++) {
    printf("<<<<<<<<<<<<<<<<<<<<<< Killing %d exec! >>>>>>>>>>>>>>>>>>>>>>\n", pidindex);
    kill(pid[pidindex], SIGKILL);
  }
  for (pidindex = 0; pidindex < 2; pidindex++) {
    printf("<<<<<<<<<<<<<<<<<<<<<< Waiting on %d exec! >>>>>>>>>>>>>>>>>>>>>>\n", pidindex);
    waitpid(pid[pidindex], NULL, 0);
    unlink(audfile[pidindex]);
  }
  pthread_exit(NULL);
  return NULL;
}

/** Gets the current state of signals for the sigframe
 *  @param sigframe
 *    the sigframe to send over in order to copy the signals
 */
void get_signal(speech_signal_t *sigframe) {
  //pthread_mutex_lock(&gst_lock);
  //memcpy(sigframe, &gst_sig, sizeof(speech_signal_t));
  //pthread_mutex_unlock(&gst_lock);
}

/** Stop recording
 */
void stop_speech_signals(void) {
  if (sslib_init) {
    sslib_exit = 1;
    pthread_join(gst_manager, NULL);
    sslib_exit = 0;
    pthread_mutex_destroy(&gst_lock);
    //stt_free(&sre);
    sslib_init = 0;
  }
  printf("stop ps...\n");
}
