Notes:
================

Use speech signal for the robot! Note that it is dependent on stt.c/o and stt,h, as well as gstreamer and the associated scripts.
input: single-channel (monaural), little-endian, unheadered 16-bit signed PCM audio file sampled at 16000 Hz

To do:
- test the joins
- double the processes
- integrate gstreamer in c
- possibly use a sigalrm scheme for faster
- create new dict
