#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/types.h>
#include "visual.h"

int result;
int pipefd[2];
FILE *cmd_output;
char buf[1024];
int status;
int procID;
int CUR_MODE;



int main() {
    char my_buff[1500];
    start_visual();
    int x = 0;
    while(1) {
        get_position(my_buff);
        printf("%d here's output: %s\n", x, my_buff);
        x++;
        if(x % 50000 == 0) {
            if(CUR_MODE == DETECT_BASKET) {
                set_detection(DETECT_BALL);
            } else {
                set_detection(DETECT_BASKET);
            }
        }
        // this simulates while loop of decision engine
    }

}
