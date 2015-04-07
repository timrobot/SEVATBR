CFLAGS = `pkg-config --cflags opencv` \
	`pkg-config --cflags gstreamer-1.0`
LIBS = `pkg-config --libs opencv` \
	`pkg-config --libs gstreamer-1.0`

TARGETS = custom_plugin

all: $(TARGETS)

custom_plugin: custom_plugin.cpp
	g++ -o $@ $< $(CFLAGS) $(LIBS)

clean:
	rm -f $(TARGETS)
