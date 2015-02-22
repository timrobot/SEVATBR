#include "stt.h"
#include <stdio.h>
#include <string.h>
#include <signal.h>

/** Initialize the speech engine
 *  @param info
 *    the information struct for the engine
 *  @return 0 on success, -1 otherwise
 */
int stt_init(stt_t *info) {
  info->config = cmd_ln_init(NULL, ps_args(), TRUE,
      "-hmm", MODELDIR "/hmm/en_US/hub4wsj_sc_8k",
      "-lm", MODELDIR "/lm/en_US/hub4.5000.DMP",
      "-dict", MODELDIR "/lm/en_US/cmu07a.dic",      // custom dictionary
      NULL);
  if (!info->config) {
    fprintf(stderr, "[stt] Cannot init config.\n");
    goto error;
  }

  info->ps = ps_init(info->config);
  if (!info->ps) {
    fprintf(stderr, "[stt] Cannot init ps.\n");
    goto error;
  }

  return 0;

error:
  memset(info, 0, sizeof(stt_t));
  return -1;
}

/** Try to decipher a file
 *  @param info
 *    the information struct for the engine
 *  @return n characters deciphered on success,
 *    -1 otherwise
 */
int stt_decipher(stt_t *info, char *filename, char **buf) {
  FILE *fh;
  char *fncpy;
  int rv;
  char const *hyp, *uttid;
  int score;

  // open file
  fh = fopen(filename, "rb");
  if (!fh) {
    fprintf(stderr, "[stt] Cannot find file: %s.\n", filename);
    return -1;
  }

  // get data
  fncpy = (char *)malloc((strlen(filename) + 1) * sizeof(char));
  strcpy(fncpy, filename);
  fncpy[strlen(fncpy) - 4] = '\0';
  rv = ps_decode_raw(info->ps, fh, fncpy, -1);
  free(fncpy);
  if (rv < 0) {
    fprintf(stderr, "[stt] Couldn't decode file: %s.\n", filename);
    return -1;
  }

  // decode
  hyp = ps_get_hyp(info->ps, &score, &uttid);
  if (!hyp) {
    fprintf(stderr, "[stt] Cannot get hypothesis\n");
    return -1;
  }

  *buf = (char *)hyp;
  fclose(fh);
  return strlen(*buf);
}

/** Remove the speech engine
 *  @param info
 *    the information struct for the engine
 */
void stt_free(stt_t *info) {
  if (info->ps) {
    ps_free(info->ps);
    memset(info, 0, sizeof(stt_t));
  }
}
