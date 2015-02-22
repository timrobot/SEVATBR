#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "iplink.h"

int main() {
  iplink_t connection;
  char *response;
  iplink_connect(&connection, "sevatbr-v002.appspot.com");
  iplink_send(&connection, "/manual_feedback", "get", NULL);
  while (!(response = iplink_recv(&connection))) ;
  printf("buffer is now: %s\n", response);
  iplink_disconnect(&connection);
  return 0;
}
