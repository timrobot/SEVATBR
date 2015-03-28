from SimpleCV import *
from time import sleep
import sys
from datetime import datetime
import numpy as np
import experiment
from particlefilter import ParticleFilter
from image_support import *

#silly globals
particle_filter = None
image_half_size = -1
save_count = 1
base_filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


def _basket_image_hue_filter(img):
    '''
    Internal wrapper image hue filter.
    @param img SimpleCV.Image the image to apply the hue filter to
    '''
    color = 280
    return image_hue_filter(img, False)

def _save_image(img):
    '''
    Saves an image to the current directory
    @param img SimpleCV.Image the image to save
    '''
    global save_count, base_filename
    f = "cam_capture_%s_%s.jpg" % (base_filename, save_count)
    img.save(f)
    print "saved image %s" % f
    save_count += 1

def _init_particle_filter(img):
    '''
    Internal wrapper to particle filter initializer.
    @param img SimpleCV.Image Any image captured from the Camera, used
    to initialize the size
    '''
    global particle_filter
    if not particle_filter:
        particle_filter = external_init_particle_filter(img)

## Single entry function returning True/False if basket is in the middle of the screen
#
# @param img SimpleCV.Image The image to test
#
def is_basket_middle(img):
    global particle_filter

    _init_particle_filter(img)
    img = _basket_image_hue_filter(img)
    blobs = get_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = get_best_blob(blobs, particle_filter)
        return is_blob_in_middle_helper(img, best)
    return False

def run_middle():
    '''
    Runs continuously and prints if the best detected blob is in the middle
    '''
    def middle_callback(img, best):
        if is_blob_in_middle_helper(img, best):
            print "Basket in middle"
    run(middle_callback)


def run(bestBlobCallback=False):
    global particle_filter

    cam = Camera()
    disp = Display()
    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        img = _basket_image_hue_filter(img)
        _init_particle_filter(img)
        blobs = get_hue_blobs(img)
        if blobs:
            particle_filter.iterate(blobs)
            #blobs.show()
            for b in blobs:
                if b.isRectangle(.2):
                    b.drawRect(color=Color.RED)
            best = get_best_blob(blobs, particle_filter)
            best.drawRect(color=Color.GREEN)
            if bestBlobCallback:
                bestBlobCallback(img, best)
        img.save(disp)
        if disp.mouseLeft:
            break
        if disp.mouseRight:
            _save_image(img)

run()
