#ifndef stt_h
#define stt_h

#ifdef __cplusplus
extern "C" {
#endif

int stt_start_listening(void);
int stt_listen(char *buffer);
void stt_stop_listening(void);

#ifdef __cplusplus
}
#endif

#endif
