CC = gcc
CFLAGS = -pedantic -Wall -std=c99 -O3
INCLUDE = -I$(shell pwd)/tbr -I$(shell pwd)/../core -I$(shell pwd)/serial
OBJECTS = robot.o

all: subdirs $(OBJECTS)

subdirs:
	make -C tbr

robot.o: robot.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

clean:
	rm -f $(OBJECTS)
