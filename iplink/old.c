#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>

int main() {
  int socket_desc;
  struct sockaddr_in server;
  char *message, server_reply[2000];

  char *hostname = "sevatbr-v002.appspot.com";
  char ip_url[100];
  struct hostent *he;
  struct in_addr **addr_list;
  int i;

  // try to get an ip address for a website
  if ((he = gethostbyname(hostname)) == NULL) {
    printf("error: get host by name\n");
    return -1;
  }
  addr_list = (struct in_addr **)he->h_addr_list;
  // get the first one
  for (i = 0; addr_list[i] != NULL; i++) {
    printf("possible connection: %s\n", inet_ntoa(*addr_list[i]));
    strcpy(ip_url, inet_ntoa(*addr_list[i]));
  }
  printf("Trying to connect to: %s\n", ip_url);

  // create socket
  socket_desc = socket(AF_INET, SOCK_STREAM, 0); // fd
  if (socket_desc == -1) {
    fprintf(stderr, "Could not create socket\n");
  }

  // define the server location, format, and port
  server.sin_addr.s_addr = inet_addr(ip_url);
  server.sin_family = AF_INET;
  server.sin_port = htons(80);

  // connect to the remote server
  if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf(stderr, "Connection error\n");
    return -1;
  } else {
    printf("Connected!\n");
  }

  // send some data - a GET request for their main page...
  message = "GET / HTTP/1.1\r\nHost: sevatbr-v002.appspot.com\r\n\r\n";
  if (send(socket_desc, message, strlen(message), 0) < 0) {
    fprintf(stderr, "failed sending the message over\n");
    return -1;
  }

  printf("Sent get request...\n\n");

  // receive some data
  if (recv(socket_desc, server_reply, 2000, 0) < 0) {
    fprintf(stderr, "failed recv'ing the message\n");
    return -1;
  }
  printf("Received: %s\n", server_reply);
  close(socket_desc);
  return 0;
}
