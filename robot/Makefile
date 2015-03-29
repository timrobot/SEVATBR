CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=c99 -O3
INCLUDE = -I$(shell pwd)/../core -I$(shell pwd)/../core/mesch -I$(shell pwd)/serial -I$(shell pwd)/tbr -I$(shell pwd)/tachikoma
SUBDIRS = $(shell pwd)/tbr $(shell pwd)/tachikoma
OBJECTS = robot.o $(shell pwd)/serial/serial.o $(shell pwd)/tbr/tbr.o $(shell pwd)/tachikoma/tachikoma.o
LIBOBJS = librobot.o $(shell pwd)/serial/libserial.o $(shell pwd)/tbr/libtbr.o $(shell pwd)/tachikoma/libtachikoma.o
TARGET = librobot.a

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

robot.o: robot.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

librobot.o: robot.c
	$(CC) $(CFLAGS) $(INCLUDE) -fPIC -o $@ -c $<

librobot.so: $(LIBOBJS) $(shell pwd)/../core/mesch/meschach.a
	$(CC) $(CFLAGS) $(INCLUDE) -shared -o $@ $^

librobot.a: $(OBJECTS) $(shell pwd)/../core/mesch/meschach.a
	ar rcs $@ $^

clean:
	rm -f $(OBJECTS) $(LIBOBJS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
