CC = g++
CFLAGS = -O3
INCLUDE = -I$(shell pwd)/speech-to-text -I$(shell pwd)/text-to-speech
OBJECTS = speech.o
SUBDIRS = speech-to-text text-to-speech

all: subdirs $(OBJECTS)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

speech.o: speech.cpp
	$(CC) $(CFLAGS) -o $@ -c $< $(INCLUDE)

clean:
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
