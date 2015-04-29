#!/usr/bin/python
import random
import time
import sys
import signal

fmode = "ball"

def switchme(signo, event):
  global fmode
  if signo == 40:
    fmode = "ball"
  if signo == 41:
    fmode = "basket"

def main():
  global fmode
  signal.signal(40, switchme)
  signal.signal(41, switchme)
  time.sleep(1)
  print "VISUAL-PROC-STARTED"
  sys.stdout.flush()
  while True:
    d = 0.0
    if fmode == "basket":
      d = 1.0
    x = (random.random() - d) * 100
    y = (random.random() - d) * 100
    z = (random.random() - d) * 100
    x = round(x * 1000) / 1000
    y = round(y * 1000) / 1000
    z = round(z * 1000) / 1000
    print "coordinate:[%s %s %s %s]" % (fmode, str(x), str(y), str(z))
    sys.stdout.flush()
    time.sleep(0.05)

main()
