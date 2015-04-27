#ifndef logger_h
#define logger_h

void log_init(void);
void log_print(char *msg);
char *log_scan(void);
void log_destroy(void);

#endif
