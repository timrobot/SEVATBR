OBJECTS = rawrec.o stt.o speech_signal.o test.o
TARGET = test

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc -g -o $@ $^ \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase` \
		`pkg-config --cflags gstreamer-0.10` `pkg-config --libs gstreamer-0.10`

rawrec.o: rawrec.c
	gcc -g -o $@ `pkg-config --cflags gstreamer-0.10` -c $<

stt.o: stt.c
	gcc -g -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase`

speech_signal.o: speech_signal.c
	gcc -g -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase` -pthread

test.o: test.c
	gcc -g -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase` -pthread

clean:
	rm -f $(TARGET) $(OBJECTS)
