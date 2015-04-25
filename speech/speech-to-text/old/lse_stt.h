#ifndef lse_stt_h
#define lse_stt_h

#ifdef __cplusplus
extern "C" {
#endif

typedef struct stt {
  char nothing;
} stt_t;

int stt_init(stt_t *info);
int stt_decipher(stt_t *info, char *filename, char **buf);
void stt_free(stt_t *info);

#ifdef __cplusplus
}
#endif

#endif
