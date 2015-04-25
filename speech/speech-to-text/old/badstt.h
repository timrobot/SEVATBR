#ifndef stt_h
#define stt_h

typedef struct stt {
  int pid;
  int bufindex;
  int fd[2];
  char buf[128];
  char phrase[128];
} stt_t;

/** Start the listening process
 *  @param info
 *    the information struct
 *  @return 0
 */
int stt_start_listening(stt_t *info);

/** Listen on pocketsphinx process,
 *  try to get the next hypothesis
 *  @param info
 *    the information struct
 *  @return a hypothesis if found, else NULL
 */
char *stt_listen(stt_t *info);

/** Stop the listening process
 *  @param info
 *    the information struct
 *  @return 0
 */
int stt_stop_listening(stt_t *info);

#endif
