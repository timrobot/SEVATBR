#include <stdio.h>
#include <stdlib.h>
#include "iplink.h"

int main() {
  iplink_t connection;
  char buffer[256];
  iplink_connect_main_server(&connection);
  iplink_send(&connection, "/manual_feedback", IPLINK_GET, NULL);
  while (iplink_recv(&connection, buffer, 256) < 1) ;
  printf("buffer is now: %s\n", buffer);
  iplink_disconnect(&connection);
  return 0;
}
