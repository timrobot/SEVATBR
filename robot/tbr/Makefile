CC = gcc
INCLUDE = -I$(shell pwd)/../serial
CFLAGS = -pedantic -Wall -Werror -std=c99 -O3
SUBDIRS = $(shell pwd)/../serial
OBJECTS = tbr.o
LIBOBJS = libtbr.o

all: subdirs $(OBJECTS) $(LIBOBJS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tbr.o: tbr.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

libtbr.o: tbr.c
	$(CC) $(CFLAGS) $(INCLUDE) -fPIC -o $@ -c $<

test.o: test.c
	$(CC) -pedantic -std=c99 -O3 $(INCLUDE) -o $@ -c $<

test: $(OBJECTS) test.o $(shell pwd)/../serial/serial.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
