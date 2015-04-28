CC = g++
CFLAGS = -pedantic -Wall -O3
INCLUDE = -I$(shell pwd) \
          -I$(shell pwd)/../user \
          -I$(shell pwd)/../agent \
          -I$(shell pwd)/../robot
LIBS = -L$(shell pwd)/../user -luser \
       -L$(shell pwd)/../agent -lagent \
	     -L$(shell pwd)/../robot -lrobot \
       -lFestival -lestools -lestbase -leststring \
       -larmadillo \
       `pkg-config --libs opencv`
SUBDIRS = $(shell pwd)/../user \
          $(shell pwd)/../agent \
          $(shell pwd)/../robot
OBJECTS = core.o
TARGET = core

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

core.o: core.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

core: core.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
