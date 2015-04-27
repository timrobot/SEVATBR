#ifndef visual_h
#define visual_h

#define DETECT_BALL 12
#define DETECT_BASKET 13

int start_visual(void);
void set_detection(int mode);
/*for now just return int of some sort, TOOD: change to use point_t*/
int get_position(char *buffer);
void stop_visual(void);

#endif
