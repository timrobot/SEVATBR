Notes:
================

Use speech signal for the robot! Note that it is dependent on stt.c/o and stt,h, as well as gstreamer and the associated scripts.
input: single-channel (monaural), little-endian, unheadered 16-bit signed PCM audio file sampled at 16000 Hz

requirements:
- gstreamer development version
- pocketsphinx development version

To do:
- test the joins
- possibly use a sigalrm scheme for faster
- create new dict (somewhat finished)
- create new adaption (http://cmusphinx.sourceforge.net/wiki/tutorialadapt)
- create shared library

Bugs:
- the core pocketsphinx library seems to segfault when not throttled - test throttle values
