CC = gcc
all: visual.o

facedetect: facedetect.cpp
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` -std=c++11

test.o: test.c
	$(CC) -o $@ -c $<

visual: visual.o test.o
	$(CC) -o $@ $^

visual.o: visual.c
	$(CC) -c visual.c

clean:
	rm -rf *.o visual
