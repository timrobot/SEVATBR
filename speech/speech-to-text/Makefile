OBJECTS = stt.o test.o
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

test.o: test.c
	gcc -o $@ -c $<

clean:
	rm -f $(TARGET) $(OBJECTS)
