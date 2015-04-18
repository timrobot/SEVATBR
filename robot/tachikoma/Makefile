CC = g++
CFLAGS = -pedantic -Wall -std=c++11 -O3
SERIAL = $(shell pwd)/../serial
ACTIONSTATE = $(shell pwd)/../ActionState
INCLUDE = -I$(SERIAL) -I $(ACTIONSTATE) -I$(shell pwd)/../../core
SUBDIRS = $(SERIAL) $(ACTIONSTATE)
SUBOBJS = $(SERIAL)/serial.o $(ACTIONSTATE)/ActionState.o
OBJECTS = tachikoma.o
LIBS = -larmadillo
TEST = test.o test

all: subdirs $(OBJECTS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tachikoma.o: tachikoma.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

xboxctrl.o:
	make -C ../../manual

test.o: test.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test: $(OBJECTS) $(SUBOBJS) test.o xboxctrl.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(SUBOBJS) $(TEST)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
