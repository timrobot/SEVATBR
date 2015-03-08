CC = gcc
INCLUDE = -I$(shell pwd)/../serial
CFLAGS = -pedantic -Wall -Werror -std=c99 -O3
LIBS = -L$(shell pwd)/../serial -lserial
OBJECTS = tbr.o test.o
TARGET = test

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	make -C $(shell pwd)/../serial
	

tbr.o: tbr.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test.o: test.c
	$(CC) -pedantic -std=c99 -O3 $(INCLUDE) -o $@ -c $<

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET)
