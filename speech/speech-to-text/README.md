on init:

  void stt_init();

to get a piece of text:

  void stt_read(char *buffer, int bytes);

in which bufer is null iff no bytes are read

to stop:

  void stt_destroy();
