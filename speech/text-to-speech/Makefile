CC = g++
INCLUDE = -I/usr/include/festival -I/usr/include/speech_tools
LIBS = -L/usr/lib/festival -lFestival -lestools -lestbase -leststring
OBJECTS = tts.o
TEST = test.o test

all: $(OBJECTS)

tts.o: tts.cpp
	$(CC) $(INCLUDE) -o $@ -c $<

test.o: test.cpp
	$(CC) -o $@ -c $<

test: $(OBJECTS) test.o
	$(CC) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TEST)
