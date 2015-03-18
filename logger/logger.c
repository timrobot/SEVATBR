#include "logger.h"

void logger_init(void);
void logger_print(char *msg);
char *logger_scan(void);
void logger_destroy(void);

void logger_init(char *logfilename) {
  printf("Logger initialized\n");
}
