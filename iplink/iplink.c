#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include "iplink.h"

static char msgbuf[1024];

int iplink_connect_main_server(iplink_t *connection) {
  struct hostent *he;
  struct in_addr **addr_list;
  struct sockaddr_in main_server;
  int i;
  connection->hostname = "sevatbr-v002.appspot.com";

  // url -> ip
  if ((he = gethostbyname(connection->hostname)) == NULL) {
    return -1;
  }
  addr_list = (struct in_addr **)he->h_addr_list;
  for (i = 0; addr_list[i] != NULL; i++) {
    strcpy(connection->ipaddr, inet_ntoa(*addr_list[i]));
  }

  // open socket connection
  if ((connection->socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    return -1;
  }
  main_server.sin_addr.s_addr = inet_addr(connection->ipaddr);
  main_server.sin_family = AF_INET;
  main_server.sin_port = htons(80); // http protocol
  if (connect(connection->socket_fd,
      (struct sockaddr *)&main_server,
      sizeof(main_server)) == -1) {
    return -1;
  }
  return 0;
}

int iplink_send(iplink_t *connection, char *addr, int type, char *data) {
  char const *typestr;
  switch (type) {
    case IPLINK_GET:
      typestr = "GET";
      break;
    case IPLINK_POST:
      typestr = "POST";
      break;
    default:
      typestr = "GET";
      break;
  }
  // TODO; put data in here for later
  sprintf(msgbuf, "%s %s HTTP/1.1\r\nHost: %s\r\n\r\n",
      typestr, addr, connection->hostname);
  return send(connection->socket_fd, msgbuf, strlen(msgbuf), 0);
}

int iplink_recv(iplink_t *connection, char *buf, int buflen) {
  return recv(connection->socket_fd, buf, buflen, 0);
}

void iplink_disconnect(iplink_t *connection) {
  close(connection->socket_fd);
}
