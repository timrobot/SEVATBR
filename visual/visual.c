#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/types.h>
#include "coord.h"
#include "visual.h"

#define VSIG_BALL   40
#define VSIG_BASKET 41

static int initd;
static int procID;
static int pipefd[2];
static char buf[256];
static int ind;
static char interim[128];
static pose3d_t loc;
static int CUR_MODE;

void set_detection(int mode) {
  switch (mode) {
    case DETECT_BALL:
      kill(procID, VSIG_BALL);
      break;

    case DETECT_BASKET:
      kill(procID, VSIG_BASKET);
      break;
  }
}

int start_visual(void) {
  if (!initd) {
    initd = 1;
    if (pipe(pipefd) == -1) {
      fprintf(stderr, "[visual] Error: cannot create the listener pipe\n");
      return -1;
    }
    procID = fork();
    if (procID == 0) {
      close(1);
      if (dup(pipefd[1]) == -1) {
        fprintf(stderr, "[visual] {child} Warning: cannot redirect output\n");
        exit(1);
      }
      printf("starting process\n");
      execlp("../visual/visual.py", "visual.py", NULL);
      printf("[visual] {child} Error: bad subprocess exec\n");
      exit(1);
    } else if (procID > 0) {
      // make read nonblocking
      int flags;
      flags = fcntl(pipefd[0], F_GETFL, 0);
      fcntl(pipefd[0], F_SETFL, flags | O_NONBLOCK);
    } else {
      fprintf(stderr, "[visual] Error: could not create subprocess\n");
      return -1;
    }
    return 0;
  }
  return -1;
}

pose3d_t *get_position(int *type) {
  int found;
  int notfound;
  int bytesread;
  char typestr[16];
  memset(typestr, 0, sizeof(typestr));
  found = 0;
  notfound = 0;
  bytesread = read(pipefd[0], interim, 127);
  while (bytesread > 0) {
    int i;
    int nbytes;
    interim[bytesread] = '\0';
    if (ind + bytesread >= sizeof(buf)) {
      memmove(buf, &buf[128], 128);
    }
    for (i = 0; i < bytesread; i++) {
      if (interim[i] == '\n') {
        if (strncmp(buf, "coordinate:", 11) == 0) {
          memset(&loc, 0, sizeof(pose3d_t));
          sscanf(buf, "coordinate:[%s %llf %llf %llf]\n", typestr, &loc.x, &loc.y, &loc.z);
          found = 1;
        }
        if (strncmp(buf, "notfound:", 9) == 0) {
          memset(&loc, 0, sizeof(pose3d_t));
          sscanf(buf, "notfound:[%s]\n", typestr);
          notfound = 1;
          printf("%s is not found\n", typestr);
        }
        ind = 0;
      } else {
        buf[ind++] = interim[i];
      }
      buf[ind] = '\0';
    }
    bytesread = read(pipefd[0], interim, 127);
  }
  if (found || notfound) {
    if (type) {
      if (strcmp(typestr, "ball") == 0) {
        *type = 1;
      } else if (strcmp(typestr, "basket") == 0) {
        *type = 2;
      } else {
        *type = 0;
      }
    }
    if (found) {
      return &loc;
    } else if (notfound) {
      return NULL;
    }
  } else {
    if (type) {
      *type = 0;
    }
    return NULL;
  }
}

void stop_visual(void) {
  if (initd) {
    close(pipefd[0]);
    close(pipefd[1]);
    kill(procID, SIGINT);
    waitpid(procID, NULL, 0);
    initd = 0;
  }
}
