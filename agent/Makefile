CC = g++
CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
INCLUDE = -I$(shell pwd)/../core
OBJECTS = agent.o
TARGETS = libagent.a

all: $(OBJECTS) $(TARGETS)

tbd.o: tbd.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

agent.o: agent.c
	$(CC) $(INCLUDE) -o $@ -c $<

libagent.a: agent.o
	ar rvs $@ $^

test.o: test.c
	gcc -o $@ -c $<

test: $(OBJECTS) libtbd.a
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS) -pthread

clean:
	rm -f $(OBJECTS) $(TARGETS)
