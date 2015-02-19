OBJECTS = serial.o comm.o test.o
TARGET = test

all: $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc -o $@ $^

serial.o: serial.c
	gcc -o $@ -c $<

comm.o: comm.c
	gcc -o $@ -c $<

test.o: test.c
	gcc -o $@ -c $<

clean:
	rm -f $(OBJECTS) $(TARGET)
