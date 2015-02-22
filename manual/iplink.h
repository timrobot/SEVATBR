#ifndef iplink_h
#define iplink_h

#ifdef __cplusplus
extern "C" {
#endif

typedef struct iplink {
  char *hostname;
  char ipaddr[128];
  int socket_fd;
  int connected;
} iplink_t;

int iplink_connect(iplink_t *connection, char *hostname);
int iplink_send(iplink_t *connection, char *addr, char *type, char *data);
char *iplink_recv(iplink_t *connection);
int iplink_disconnect(iplink_t *connection);

#ifdef __cplusplus
}
#endif

#endif
