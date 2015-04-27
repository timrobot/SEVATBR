#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include "visual.h"

int result;
int pipefd[2];
FILE *cmd_output;
char buf[1024];
int status;
int procID;

int start_visual(void) {

    result = pipe(pipefd);
    if (result < 0) {
        perror("pipe");
        exit(-1);
    }

    result = fork();
    if(result < 0) {
        exit(-1);
    }
    procID = result;

    if (result == 0) {
        dup2(pipefd[1], STDOUT_FILENO); /* Duplicate writing end to stdout */
        close(pipefd[0]);
        close(pipefd[1]);

        execl("./visual.py", "doesntmatter", NULL);
        _exit(1);
    } else if (result > 0){
        int flags;
      flags = fcntl(pipefd[0], F_GETFL, 0);
      fcntl(pipefd[0], F_SETFL, flags | O_NONBLOCK);
    }

}

int main() {
    char my_buff[1500];
    start_visual();
    int x = 0;
    while(1) {
        get_position(my_buff);
        printf("%d here's output: %s\n", x, my_buff);
        x++;
        // this simulates while loop of decision engine
    }

}

int get_position(char * in_buffer) {

    /* Parent process */
    close(pipefd[1]); /* Close writing end of pipe */

    cmd_output = fdopen(pipefd[0], "r");

    if(fgets(buf, sizeof buf, cmd_output)) {
        if (strlen(buf) > 0) {
            strcpy(in_buffer, buf);
            return strlen(buf);
        }
    } 

    return 0;
}

void stop_visual(void) {
    close(pipefd[0]);
    close(pipefd[1]);
    kill(procID, SIGINT);
    waitpid(procID, NULL, 0);
}