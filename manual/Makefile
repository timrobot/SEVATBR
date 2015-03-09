CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=c99 -O3
INCLUDE = -I$(shell pwd)/../core
OBJECTS = httplink.o manual.o test.o
TARGET = test

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) -o $@ $^ $(INCLUDE)

manual.o: manual.c
	$(CC) -o $@ -c $< $(INCLUDE)

httplink.o: httplink.c
	$(CC) -o $@ -c $<

test.o: test.c
	$(CC) -o $@ -c $< $(INCLUDE)

clean:
	rm -f $(OBJECTS) $(TARGET)
