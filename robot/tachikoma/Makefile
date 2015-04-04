CC = g++
CFLAGS = -pedantic -Wall -O3
INCLUDE = -I$(shell pwd)/../serial -I$(shell pwd)/../../core
SUBDIRS = $(shell pwd)/../serial
OBJECTS = tachikoma.o
LIBOBJS = libtachikoma.o
TEST = test.o test

all: subdirs $(OBJECTS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tachikoma.o: tachikoma.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

libtachikoma.o: tachikoma.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -fPIC -o $@ -c $<

test.o: test.cpp
	$(CC) -pedantic -std=c99 -O3 $(INCLUDE) -o $@ -c $<

test: $(OBJECTS) test.o $(shell pwd)/../serial/serial.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^

clean:
	rm -f $(OBJECTS) $(LIBOBJS) $(TEST)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
