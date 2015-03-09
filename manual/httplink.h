#ifndef httplink_h
#define httplink_h

#ifdef __cplusplus
extern "C" {
#endif

typedef struct httplink {
  char *hostname;
  char ipaddr[128];
  int socket_fd;
  int connected;
} httplink_t;

int httplink_connect(httplink_t *connection, char *hostname);
int httplink_send(httplink_t *connection, char *addr, char *type, char *data);
char *httplink_recv(httplink_t *connection);
int httplink_disconnect(httplink_t *connection);

#ifdef __cplusplus
}
#endif

#endif
