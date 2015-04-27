CC = gcc
all:
	g++ -o detect facedetect.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv` -std=c++11

visual: visual.o
	$(CC) -o visual visual.o

visual.o: visual.c
	$(CC) -c visual.c

clean:
	rm -rf *.o visual
