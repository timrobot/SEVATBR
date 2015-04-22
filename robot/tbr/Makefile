CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=gnu99 -O3
INCLUDE = -I$(shell pwd)/../serial
SUBDIRS = $(shell pwd)/../serial
OBJECTS = tbr.o
TEST = test.o test

all: subdirs $(OBJECTS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tbr.o: tbr.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test.o: test.c
	$(CC) -pedantic -std=c99 -O3 $(INCLUDE) -o $@ -c $<

test: $(OBJECTS) test.o $(shell pwd)/../serial/serial.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^

clean:
	rm -f $(OBJECTS) $(TEST)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
