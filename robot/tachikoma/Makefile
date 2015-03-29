CC = gcc
CFLAGS = -pedantic -Wall -std=gnu99 -O3
INCLUDE = -I$(shell pwd)/../serial -I$(shell pwd)/../../core -I$(shell pwd)/../../core/mesch
SUBDIRS = $(shell pwd)/../serial
OBJECTS = tachikoma.o
LIBOBJS = libtachikoma.o
TEST = test.o test

all: subdirs $(OBJECTS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tachikoma.o: tachikoma.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

libtachikoma.o: tachikoma.c
	$(CC) $(CFLAGS) $(INCLUDE) -fPIC -o $@ -c $<

test.o: test.c
	$(CC) -pedantic -std=c99 -O3 $(INCLUDE) -o $@ -c $<

test: $(OBJECTS) test.o $(shell pwd)/../serial/serial.o $(shell pwd)/../../core/mesch/mesch.a
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^

clean:
	rm -f $(OBJECTS) $(LIBOBJS) $(TEST)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
