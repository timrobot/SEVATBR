CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=gnu99 -O3
INCLUDE = -I$(shell pwd)/../core
OBJECTS = httplink.o xboxctrl.o user.o
TARGET = libuser.a
TEST = test.o test

all: $(OBJECTS) $(TARGET)

httplink.o: httplink.c
	$(CC) $(CFLAGS) -o $@ -c $<

xboxctrl.o: xboxctrl.c
	$(CC) $(CFLAGS) -o $@ -c $<

user.o: user.c
	$(CC) $(CFLAGS) -o $@ -c $< $(INCLUDE)

libuser.a: $(OBJECTS)
	ar rcs $@ $^

test.o: test.c
	$(CC) -o $@ -c $< $(INCLUDE)

test: $(OBJECTS) test.o
	$(CC) $(CFLAGS) -o $@ $^ $(INCLUDE)

clean:
	rm -f $(OBJECTS) $(TARGET) $(TEST)
