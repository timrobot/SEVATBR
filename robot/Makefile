CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=c99 -O3
INCLUDE = -I$(shell pwd)/tbr -I$(shell pwd)/../core -I$(shell pwd)/serial
SUBDIRS = $(shell pwd)/tbr 
OBJECTS = robot.o $(shell pwd)/serial/serial.o $(shell pwd)/tbr/tbr.o
LIBOBJS = librobot.o $(shell pwd)/serial/libserial.o $(shell pwd)/tbr/libtbr.o
TARGET = librobot.so librobot.a

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

robot.o: robot.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

librobot.o: robot.c
	$(CC) $(CFLAGS) $(INCLUDE) -fPIC -o $@ -c $<

librobot.so: $(LIBOBJS)
	$(CC) $(CFLAGS) $(INCLUDE) -shared -o $@ $^

librobot.a: $(OBJECTS)
	ar rcs $@ $^

clean:
	rm -f $(OBJECTS) $(LIBOBJS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
