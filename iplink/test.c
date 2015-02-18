#include "iplink.h"

int main() {
  iplink_t connection;
  iplink_connect_main_server(&connection);
  iplink_disconnect(&connection);
  return 0;
}
