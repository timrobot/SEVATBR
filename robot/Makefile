CC = g++
CFLAGS = -pedantic -Wall -Werror --std=c++11 -O3
INCLUDE = -I$(shell pwd)/../core \
          -I$(shell pwd)/serial \
          -I$(shell pwd)/tbr
SUBDIRS = $(shell pwd)/tbr
OBJECTS = robot.o
LIBOBJS = $(shell pwd)/serial/serial.o \
          $(shell pwd)/tbr/tbr.o
TARGET = librobot.a

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

robot.o: robot.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

librobot.a: $(OBJECTS) $(LIBOBJS)
	ar rvs $@ $^

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
