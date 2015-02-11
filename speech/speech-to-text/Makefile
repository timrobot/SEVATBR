OBJECTS = stt.o test.o speech_signal.o
TARGET = test

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc -o $@ $^ \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase`

stt.o: stt.c
	gcc -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase`

speech_signal.o: speech_signal.c
	gcc -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase` -pthread

test.o: test.c
	gcc -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase` -pthread

clean:
	rm -f $(TARGET) $(OBJECTS)
