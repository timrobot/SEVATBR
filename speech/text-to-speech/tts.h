#ifndef tts_h
#define tts_h

#ifdef __cplusplus
extern "C" {
#endif

int tts_select_voice(const char *voicename);
int tts_say(const char *msg);

#ifdef __cplusplus
}
#endif

#endif
