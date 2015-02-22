OBJECTS = iplink.o test1.o manual.o test2.o
TARGET = test1 test2

all: $(OBJECTS) $(TARGET)

test2: iplink.o manual.o test2.o
	gcc -o $@ $^ -I$(shell pwd)/../robot

test1: iplink.o test1.o
	gcc -o $@ $^

manual.o: manual.c
	gcc -o $@ -c $< -I$(shell pwd)/../robot

iplink.o: iplink.c
	gcc -o $@ -c $<

test1.o: test1.c
	gcc -o $@ -c $<

test2.o: test2.c
	gcc -o $@ -c $< -I$(shell pwd)/../robot

clean:
	rm -f $(OBJECTS) $(TARGET)
