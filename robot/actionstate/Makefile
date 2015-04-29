CC = g++
CFLAGS = -pedantic -Wall --std=c++11 -O3
LIBS = -larmadillo
OBJECTS = actionstate.o

all: $(OBJECTS)

actionstate.o: actionstate.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJECTS)
