CFLAGS=-Wall -g -O3 $(shell pkg-config --cflags opencv)
CFLAGS+=$(shell pkg-config --cflags opencv)
LDFLAGS=$(shell pkg-config --libs opencv)

ball-rec: ball-rec.c
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f ball-rec
