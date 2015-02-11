#ifndef rawrec_h
#define rawrec_h

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rawrec {
  void *pipeline;
  void *source;
  void *sink;
  void *buffer;
  void *caps;
  char *filesink_loc;
} rawrec_t;

int start_recording(rawrec_t *rr, char *fsinkloc);
void stop_recording(rawrec_t *rr);

#ifdef __cplusplus
}
#endif

#endif
