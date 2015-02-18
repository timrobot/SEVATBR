#ifndef iplink_h
#define iplink_h

#define IPLINK_GET  0x01
#define IPLINK_POST 0x02

typedef struct iplink {
  char *hostname;
  char ipaddr[100];
  int socket_fd;
} iplink_t;

int iplink_connect_main_server(iplink_t *connection);
int iplink_send(iplink_t *connection, char *addr, int type, char *data);
int iplink_recv(iplink_t *connection, char *buf, int buflen);
void iplink_disconnect(iplink_t *connection);

#endif
