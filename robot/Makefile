CC = g++
CFLAGS = -pedantic -Wall -Werror -O3
INCLUDE = -I$(shell pwd)/../core -I$(shell pwd)/serial -I$(shell pwd)/tbr -I$(shell pwd)/tachikoma
SUBDIRS = $(shell pwd)/tbr $(shell pwd)/tachikoma
OBJECTS = robot.o $(shell pwd)/serial/serial.o $(shell pwd)/tbr/tbr.o $(shell pwd)/tachikoma/tachikoma.o
LIBS = -larmadillo
TARGET = librobot.a

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

robot.o: robot.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

librobot.a: $(OBJECTS) $(LIBS)
	ar rcs $@ $^

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
