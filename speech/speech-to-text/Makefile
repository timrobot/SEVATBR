OBJECTS = stt.o test.o speech_signal.o
TARGET = test

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc -g -o $@ $^ \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase`

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

gst_play: gst_play.c
	gcc -g -o $@ `pkg-config --cflags glib-2.0` `pkg-config --libs glib-2.0` \
		`pkg-config --cflags gstreamer-0.10` `pkg-config --libs gstreamer-0.10` $<

clean:
	rm -f $(TARGET) $(OBJECTS)
