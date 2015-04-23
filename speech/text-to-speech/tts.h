#ifndef tts_h
#define tts_h

#ifdef __cplusplus
extern "C" {
#endif

int tts_select_voice(char const *voicename);
int tts_say(char const *msg);

#ifdef __cplusplus
}
#endif

#endif
