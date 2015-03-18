#include "logger.h"

void logger_init(char *filename);
void logger_print(char *msg);
char *logger_scan(void);
void logger_destroy(void);

static FILE *logfile;

void logger_init(char *fname) {
  if (logfile) {
    logger_destroy();
  }
  if (!fname) {
    return;
  }
  logfile = fopen(fname, "w+");
}

void logger_print(char *msg) {
  fprintf(logfile, msg);
}

char *logger_scan(void) {
  if (logfile) {
    long start, end;
    char *entire_msg;
    end = ftell(logfile);
    fseek(logfile, 0L, SEEK_SET);
    start = ftell(logfile);
    entire_msg = (char *)malloc(end - start + 1);
    fread((void *)entire_msg, 1, end - start, logfile);
    entire_msg[end - start] = '\0';
    fseek(logfile, 0L, SEEK_END);
    return entire_msg;
  } else {
    return NULL;
  }
}

void logger_destroy(void) {
  if (logfile) {
    fclose(logfile);
    logfile = NULL;
  }
}
