CC = g++
CFLAGS = -pedantic -Wall -O3 `pkg-config --cflags opencv`
INCLUDE = -I$(shell pwd) -I$(shell pwd)/../manual -I$(shell pwd)/../robot -I$(shell pwd)/../agent
LIBS = $(shell pwd)/../manual/libmanual.a \
	$(shell pwd)/../robot/librobot.a \
	$(shell pwd)/../agent/libagent.a \
	`pkg-config --libs opencv`
SUBDIRS = $(shell pwd)/../manual $(shell pwd)/../robot $(shell pwd)/../agent
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
