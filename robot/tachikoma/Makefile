CC = g++
CFLAGS = -pedantic -Wall --std=c++11 -O3
SERIAL = $(shell pwd)/../serial
ACTIONSTATE = $(shell pwd)/../actionstate
USER = $(shell pwd)/../../user
CORE = $(shell pwd)/../../core
INCLUDE = -I$(SERIAL) -I$(ACTIONSTATE) -I$(USER) -I$(CORE)
SUBDIRS = $(SERIAL) $(ACTIONSTATE) $(USER)
SUBOBJS = $(SERIAL)/serial.o $(ACTIONSTATE)/actionstate.o $(USER)/xboxctrl.o
OBJECTS = tachikoma.o
LIBS = -larmadillo
TEST = test.o test stage1test.o stage1test

all: subdirs $(OBJECTS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tachikoma.o: tachikoma.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test.o: test.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test: $(OBJECTS) $(SUBOBJS) test.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

stage1test.o: stage1test.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

stage1test: $(OBJECTS) $(SUBOBJS) stage1test.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(SUBOBJS) $(TEST)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
