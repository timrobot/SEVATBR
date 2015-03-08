CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=gnu99 -O3
OBJECTS = serial.o libserial.o
TARGET = libserial.so

all: $(OBJECTS) $(TARGET)

serial.o: serial.c
	$(CC) $(CFLAGS) -o $@ -c $<

libserial.o: serial.c
	$(CC) $(CFLAGS) -fPIC -o $@ -c $<

libserial.so: libserial.o
	$(CC) $(CFLAGS) -shared -o $@ $<

clean:
	rm -f $(OBJECTS) $(TARGET)
