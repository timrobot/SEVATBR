#include "logger.h"

void logger_init(char *filename);
void logger_print(char *msg);
char *logger_scan(void);
void logger_destroy(void);

static FILE *logfile;

/** Start the logger
 *  @param fname
 *    name of the input file
 */
void logger_init(char *fname) {
  if (logfile) {
    logger_destroy();
  }
  if (!fname) {
    return;
  }
  logfile = fopen(fname, "w+");
}

/** Print something to the logger
 *  @param msg
 *    the message to print to the file
 */
void logger_print(char *msg) {
  if (!logfile) {
    return;
  }
  fprintf(logfile, msg);
}

/** Get the current logger file
 *  @return the entire message on success, NULL otherwise
 */
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

/** Stop the logger
 */
void logger_destroy(void) {
  if (logfile) {
    fclose(logfile);
    logfile = NULL;
  }
}
