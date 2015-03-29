OBJECTS = rawrec.o stt.o speech_signal.o test.o
TARGET = test

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc -O3 -o $@ $^ \
		`pkg-config --cflags gstreamer-0.10` `pkg-config --libs gstreamer-0.10` \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase`

rawrec.o: rawrec.c
	gcc -O3 -o $@ `pkg-config --cflags gstreamer-0.10` -c $<

stt.o: stt.c
	gcc -O3 -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase`

speech_signal.o: speech_signal.c
	gcc -O3 -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase` -pthread

test.o: test.c
	gcc -O3 -o $@ -c $< \
		-I/usr/include/pocketsphinx/ \
		-DMODELDIR=\"`pkg-config --variable=modeldir pocketsphinx`\" \
		`pkg-config --cflags --libs pocketsphinx sphinxbase` -pthread

clean:
	rm -f $(TARGET) $(OBJECTS)
