CC = g++
CFLAGS = -fPIC -g -Wall
INCLUDE = -I/usr/local/include/libfreenect -I/usr/include/libusb-1.0
LIBS = -L/usr/local/lib -lfreenect -pthread -larma
OBJECTS = kinect.o
TEST = test.o test

all: $(OBJECTS)

kinect.o: kinect.cpp
	$(CC) -c $(CFLAGS) kinect.cpp -o kinect.o $(INCLUDE)

test.o: test.cpp
	$(CC) $(CFLAGS) -c $^ -o $@ $(INCLUDE)

test: test.o kinect.o
	$(CC) $(INCLUDE) $(CFLAGS) -o $@ $< $(LIBS)

clean:
	rm -f $(OBJECTS) $(TEST)
