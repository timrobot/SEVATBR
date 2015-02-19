OBJECTS = iplink.o test.o
TARGET = iplink

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc -o $@ $^

iplink.o: iplink.c
	gcc -o $@ -c $<

test.o: test.c
	gcc -o $@ -c $<

clean:
	rm -f $(OBJECTS) $(TARGET)
