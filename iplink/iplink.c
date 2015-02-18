#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include "iplink.h"

static char msgbuf[1024];

/** Connect to the main server for our robot's manual interface.
 *  @param connection
 *    the connection information for the server
 *  @return 0 on success, -1 otherwise
 */
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

/** Send a request to the main server
 *  @param connection
 *    the connection information for the server
 *  @param addr
 *    the addr to send the request to, "/" for main page
 *  @param type
 *    either get (IPLINK_GET) or post (IPLINK_POST)
 *  @param data
 *    the data to send over (only for IPLINK_POST)
 *  @return n bytes sent over, -1 otherwise
 */
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

/** Try and receive a response from the main server
 *  @param connection
 *    the connection information for the server
 *  @param buf
 *    the buffer to store the message in
 *  @param buflen
 *    the max amount of data to write to the buffer
 *  @return n bytes received, -1 otherwise
 */
int iplink_recv(iplink_t *connection, char *buf, int buflen) {
  return recv(connection->socket_fd, buf, buflen, 0);
}

/** Disconnect the connection
 *  @param connection
 *    the connection information for the server
 */
void iplink_disconnect(iplink_t *connection) {
  close(connection->socket_fd);
}
