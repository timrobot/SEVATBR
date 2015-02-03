#include <stdio.h>
#include <string.h>
#include "stt.h"

int main(int argc, char **argv) {
  stt_t info;
  char *buf;

  if (argc != 2) {
    printf("%s [audio file]\n", argv[0]);
    return -1;
  }

  if (stt_init(&info) == -1) {
    fprintf(stderr, "Failure at init\n");
    return -1;
  }

  if (stt_decipher(&info, argv[1], &buf) == -1) {
    fprintf(stderr, "Failure at decipher\n");
    return -1;
  } else {
    printf("Deciphered: %s\n", buf);
  }

  stt_free(&info);
  return 0;
}
