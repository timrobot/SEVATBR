#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include "stt.h"

static const char *blacklist[] = { "\n", NULL };
static int blacklist_contains(char *phrase);

/** Start the listening process
 *  @param info
 *    the information struct
 *  @return 0
 */
int stt_start_listening(stt_t *info) {
  if (pipe(info->fd) == -1) {
    return -1;
  }
  info->pid = fork();
  if (info->pid == -1) {
    fprintf(stderr, "[stt] Error: could not create a process for pocketsphinx\n");
    return -1;
  } else if (info->pid == 0) {
    close(1);
    if (dup(info->fd[1]) == -1) {
      fprintf(stderr, "[stt] {child} Error: pipe creation from child to parent [1] failed\n");
      exit(1);
    }
    execlp("./pocketsphinx_decipher.sh", "pocketsphinx_decipher.sh", NULL);
    fprintf(stderr, "[stt] Error: could not init the pocksetsphinx decipher script\n");
    exit(2);
  } else {
    int flags;
    flags = fcntl(info->fd[0], F_GETFL, 0);
    fcntl(info->fd[0], F_SETFL, flags | O_NONBLOCK); // make the read nonblocking
  }
  info->bufindex = 0;
  memset(info->buf, 0, sizeof(char) * 128);
  memset(info->phrase, 0, sizeof(char) * 128);
  return 0;
}

/** Listen on pocketsphinx process,
 *  try to get the next hypothesis
 *  @param info
 *    the information struct
 *  @return a hypothesis if found, else NULL
 */
char *stt_listen(stt_t *info) {
  char c;
  int foundphrase;
  if (read(info->fd[0], &c, 1) < 1) { // read from stdin
    return NULL;
  }
  printf("found: %c\n", c);
  info->buf[info->bufindex++] = c;
  foundphrase = 0;
  if (c == '\n') { // on new line
    info->buf[info->bufindex] = '\0';
    printf("[[[[[ BUF ]]]]]: {{ %s }}\n", info->buf);
    if (strncmp(info->buf, "READY", 5) == 0) { // hypothesis indicator
      printf("<<<<<<<<<<<<<<<<<<<<<<<<<< found: %s\n", info->phrase);
      foundphrase = strlen(info->phrase) != 0;
    } else {
      // store the previous phrase
      /*if (!blacklist_contains(info->buf)) {
        strncpy(info->phrase, info->buf, info->bufindex);
        info->phrase[info->bufindex - 1] = '\0'; // remove the new line
      } else {
        info->phrase[0] = '\0'; // reset to an empty phrase
      }*/
      strncpy(info->phrase, info->buf, info->bufindex);
      info->phrase[info->bufindex - 1] = '\0';
    }
    info->bufindex = 0; // start over
  }
  return foundphrase ? info->phrase : NULL;
}

/** Stop the listening process
 *  @param info
 *    the information struct
 *  @return 0
 */
int stt_stop_listening(stt_t *info) {
  close(info->fd[0]);
  close(info->fd[1]);
  kill(info->pid, SIGINT);
  waitpid(info->pid, NULL, 0);
  memset(info, 0, sizeof(stt_t));
  return 0;
}

/** Find out if the phrase is in the blacklist
 *  @param phrase
 *    the phrase to check
 *  @return whether or not the phrase is in the blacklist
 */
static int blacklist_contains(char *phrase) {
  int i;
  const char *str;
  for (i = 0, str = blacklist[0]; str != NULL; str = blacklist[++i]) {
    if (strncmp(str, phrase, strlen(str)) == 0) {
      return 1;
    }
  }
  return 0;
}
