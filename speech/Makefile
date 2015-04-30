CC = g++
CFLAGS = -pedantic -Wall -O3
INCLUDE = -I$(shell pwd)/text-to-speech
          #-I$(shell pwd)/speech-to-text
SUBDIRS = $(shell pwd)/text-to-speech #$(shell pwd)/speech-to-text 
OBJECTS = speech.o
LIBOBJS = $(shell pwd)/text-to-speech/tts.o
          #$(shell pwd)/speech-to-text/stt.o
TARGET = libspeech.a

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

speech.o: speech.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

libspeech.a: $(OBJECTS) $(LIBOBJS)
	ar rvs $@ $^

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
