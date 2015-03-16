CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=gnu99 -O3
INCLUDE = -I$(shell pwd)/../core
OBJECTS = httplink.o controller.o manual.o
LIBOBJS = libhttplink.o libmanual.o
TARGET = libmanual.so libmanual.a
TEST = test.o test

all: $(OBJECTS) $(TARGET)

httplink.o: httplink.c
	$(CC) $(CFLAGS) -o $@ -c $<

libhttplink.o: httplink.c
	$(CC) $(CFLAGS) -fPIC -o $@ -c $<

controller.o: controller.c
	$(CC) $(CFLAGS) -pthread -o $@ -c $<

libcontroller.o: controller.c
	$(CC) $(CFLAGS) -pthread -fPIC -o $@ -c $<

manual.o: manual.c
	$(CC) $(CFLAGS) -o $@ -c $< $(INCLUDE)

libmanual.o: manual.c
	$(CC) $(CFLAGS) -fPIC -o $@ -c $< $(INCLUDE)
	
libmanual.so: $(LIBOBJS)
	$(CC) $(CFLAGS) -shared -o $@ $^

libmanual.a: $(OBJECTS)
	ar rcs $@ $^

test.o: test.c
	$(CC) -o $@ -c $< $(INCLUDE)

test: $(OBJECTS) test.o
	$(CC) $(CFLAGS) -pthread -o $@ $^ $(INCLUDE)

clean:
	rm -f $(OBJECTS) $(LIBOBJS) $(TARGET) $(TEST)
