#include "tts.h"

int main() {
  tts_select_voice("voice_kal_diphone");
  tts_say("I went to the bathroom "
          "There was no toilet paper "
          "I had to go under the kitchen sink"
          "in order to grab a new roll.");
  return 0;
}
