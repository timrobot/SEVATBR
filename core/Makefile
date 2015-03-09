CC = gcc
CFLAGS = -pedantic -Wall -std=c99 -O3
INCLUDE = -I$(shell pwd) -I$(shell pwd)/../manual -I$(shell pwd)/../robot
LIBS = -L$(shell pwd)/../manual -lmanual -L$(shell pwd)/../robot -lrobot
SUBDIRS = $(shell pwd)/../manual $(shell pwd)/../robot
OBJECTS = core.o
TARGET = core

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

core.o: core.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

core: core.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
