#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include "lse_stt.h"
#define NTARGETS 2

typedef union {
  struct {
    char low;
    char high;
  };
  int16_t val;
} note;


static char *db[NTARGETS];
static long length[NTARGETS];
static int16_t *dbfiles[NTARGETS];
static int16_t *loadFile(char *fname, long *s);
static unsigned long long square_error(int16_t *haystack, int hlen, int16_t *needle, int nlen, int *start);
static char *text[NTARGETS];
static char goodtogo;

int stt_init(stt_t *info) {
  int i;
  db[0] = "newreturn.raw";
  db[1] = "newfetch.raw";
  for (i = 0; i < NTARGETS; i++) {
    if (access(db[i], F_OK) == -1) {
      return -1;
    }
  }
  goodtogo = 1;
  for (i = 0; i < NTARGETS; i++) {
    dbfiles[i] = loadFile(db[i], &length[i]);
  }
  text[0] = "return";
  text[1] = "fetch";
  return 0;
}


int16_t *loadFile(char *fname, long *s) {
  FILE *fp = fopen(fname, "r");
  long start = ftell(fp);
  if (fp == NULL) return NULL;
  fseek(fp, 0L, SEEK_END);
  long end = ftell(fp);
  long size = (end - start) / 2L;
  if (size == 0) {
    return NULL;
  }
  int16_t *notes = malloc(sizeof(int16_t) * size);
  fseek(fp, 0L, SEEK_SET);
  int i = 0;
  for (i = 0; i < size; i++) {
    note n;
    n.low = fgetc(fp);
    n.high = fgetc(fp);
    notes[i] = n.val;
  }
  *s = size;
  printf("[FILE] loaded: %s with size %ld\n", fname, size);
  return notes;
}

unsigned long long square_error(int16_t *haystack, int hlen, int16_t *needle, int nlen, int *start) {
  unsigned long long sqerr = ((unsigned long long)-1);
  int index = -1;
  int i, j;
  printf("eval the sqerr for HAYSTACK LEN: %d, NEEDLE LEN: %d\n", hlen, nlen);
  for (i = 0; i < hlen - nlen; i++) {
    unsigned long long interim = 0L;
    for (j = 0; j < nlen; j++) {
      unsigned long long err = haystack[i + j] - needle[i];
      err = err * err *err;
      interim += err;
    }
    if (interim < sqerr) {
      sqerr = interim;
      index = i;
    }
  }
  *start = index;
  printf("done eval\n");
  return sqerr;
}

int stt_decipher(stt_t *info, char *fname, char **buf) {
  if (!goodtogo) {
    *buf = NULL;
    return -1;
  }
  printf("trying to decipher: %s\n", fname);
  long l;
  int i;
  int16_t *phrase = loadFile(fname, &l);
  if (l == -1) {
    *buf = NULL;
    return -1;
  }
  unsigned long long err[NTARGETS];
  int index[NTARGETS];
  for (i = 0; i < NTARGETS; i++) {
    err[i] = square_error(phrase, l, dbfiles[i], length[i], &index[i]);
    printf("err[%d]: %lld\n", i, err[i]);
  }

  free(phrase);

  unsigned long long lowest_err = ((unsigned long long)-1);
  int li = -1;
  for (i = 0; i < NTARGETS; i++) {
    li = i;
    lowest_err = err[i];
  }

  if (lowest_err < 100000000) {
    *buf = text[li];
    return strlen(text[li]);
  } else {
    *buf = NULL;
    return -1;
  }
}

void stt_free(stt_t *info) {
  if (!goodtogo) {
    return;
  }
  int i;
  for (i = 0; i < NTARGETS; i++) {
    if (dbfiles[i]) {
      free(dbfiles[i]);
      dbfiles[i] = NULL;
    }
  }
  goodtogo = 0;
}
