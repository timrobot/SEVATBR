CC = g++
CFLAGS = -pedantic -Wall -std=c++11 -O3
LIBS = -larmadillo
OBJECTS = ActionState.o

all: $(OBJECTS)

ActionState.o: ActionState.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJECTS)
