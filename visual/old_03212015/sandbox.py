'''
Small example of SimpleCV blob detection
'''

from SimpleCV import *
from time import sleep
import sys

cam = Camera()
disp = Display()

while disp.isNotDone():
    sleep(.05)
    img = cam.getImage()

    #close window with left click
    if disp.mouseLeft:
        break

    #match blobs in color range
    mask = img.createBinaryMask(color1=(120,0,0),color2=(255,105,105))
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)

    #draw the largest blob
    largest_blob = None
    if blobs is not None:
        for b in blobs:
            if largest_blob is None:
                largest_blob = b
            elif b.area() > largest_blob.area():
                largest_blob = b
    if(largest_blob is not None):
        img.drawRectangle(largest_blob.minX(),largest_blob.minY(),largest_blob.width(),largest_blob.height(),color=(0,255,0))

    img.save(disp)
