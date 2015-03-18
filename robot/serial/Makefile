CC = gcc
CFLAGS = -pedantic -Wall -std=gnu99 -O3
OBJECTS = serial.o
LIBOBJS = libserial.o

all: $(OBJECTS) $(LIBOBJS)

serial.o: serial.c
	$(CC) $(CFLAGS) -o $@ -c $<

libserial.o: serial.c
	$(CC) $(CFLAGS) -fPIC -o $@ -c $<

clean:
	rm -f $(OBJECTS) $(LIBOBJS)
