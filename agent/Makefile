CC = g++
CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
CORE = $(shell pwd)/../core
VISUAL = $(shell pwd)/../visual
SPEECH = $(shell pwd)/../speech
ROBOT = $(shell pwd)/../robot
SUBDIRS = $(VISUAL) $(SPEECH) $(ROBOT)
INCLUDE = -I$(CORE) -I$(VISUAL) -I$(SPEECH) -I$(ROBOT)
OBJECTS = agent.o $(VISUAL)/visual.o $(SPEECH)/speech.o $(ROBOT)/robot.o
TARGETS = libagent.a

all: subdirs $(OBJECTS) $(TARGETS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

tbd.o: tbd.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

agent.o: tbr_agent.cpp
	$(CC) $(INCLUDE) -o $@ -c $<

libagent.a: $(OBJECTS)
	ar rvs $@ $^

test.o: test.c
	gcc -o $@ -c $<

test: $(OBJECTS) libtbd.a
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS) -pthread

clean:
	rm -f $(OBJECTS) $(TARGETS)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
