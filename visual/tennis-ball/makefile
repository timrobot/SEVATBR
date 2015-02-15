CC=gcc
CFLAGS+=`pkg-config --cflags opencv`
LDFLAGS+=`pkg-config --libs opencv`

PROG=ball-rec
OBJS=$(PROG).o

.PHONY: all clean
	$(PROG): $(OBJS)
	    $(CC) -o $(PROG) $(OBJS) $(LDFLAGS) $(CLFAGS)

%.o: %.c
	    $(CC) -c $(CFLAGS) $<

all: $(PROG)

clean:
	    rm -f $(OBJS) $(PROG)
