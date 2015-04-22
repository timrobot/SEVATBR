CC = gcc
CFLAGS = -pedantic -Wall -std=gnu99 -O3
OBJECTS = serial.o

all: $(OBJECTS)

serial.o: serial.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJECTS)
