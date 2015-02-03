#ifndef stt_h
#define stt_h

#include <pocketsphinx.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct stt {
  ps_decoder_t *ps;
  cmd_ln_t *config;
} stt_t;

int stt_init(stt_t *info);
int stt_decipher(stt_t *ino, char *filename, char **buf);
void stt_free(stt_t *info);

#ifdef __cplusplus
}
#endif

#endif
