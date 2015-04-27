CC = g++
CFLAGS = -pedantic -Wall -O3 `pkg-config --cflags opencv`
INCLUDE = -I$(shell pwd) -I$(shell pwd)/../user -I$(shell pwd)/../robot #-I$(shell pwd)/../agent
LIBS = $(shell pwd)/../user/libuser.a \
	$(shell pwd)/../robot/librobot.a \
	`pkg-config --libs opencv` \
	-larmadillo
#	$(shell pwd)/../agent/libagent.a
SUBDIRS = $(shell pwd)/../user $(shell pwd)/../robot #$(shell pwd)/../agent
OBJECTS = core.o
TARGET = core

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

core.o: core.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

core: core.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
