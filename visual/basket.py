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

def basket_image_filter(img):
    '''
    CURRENTLY NOT BEING USED.
    '''
    #removes very-red reds
    removal_mask = img.createBinaryMask(color1=(70,0,0), color2=(255,255,255)).invert()
    img = img.applyBinaryMask(removal_mask)

    #removes less-red reds if they don't have enough blue
    removal_mask = img.createBinaryMask(color1=(30,0,0), color2=(70,255,100)).invert()
    img = img.applyBinaryMask(removal_mask)

    #removes very-green greens
    removal_mask = img.createBinaryMask(color1=(0,100,0), color2=(255,255,255)).invert()
    img = img.applyBinaryMask(removal_mask)

    #removes very small reds if it has little blue
    removal_mask = img.createBinaryMask(color1=(0,0,0), color2=(40,255,35)).invert()
    img = img.applyBinaryMask(removal_mask)
    return img

def _basket_image_hue_filter(img):
    '''
    Internal wrapper image hue filter.
    '''
    color = 280
    return image_hue_filter(img, False)

def _get_basket_blobs(img):
    '''
    Gets basket blobs with RGB range

    CURRENTLY NOT BEING USED.
    '''
    # accepted color range
    start_color = (0,0,35)
    end_color = (255,255,255)

    # create binary mask based on color ranges
    mask = img.createBinaryMask(color1=start_color,color2=end_color)

    # find binary blobs in the image
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs

def _save_image(img):
    global save_count, base_filename
    f = "cam_capture_%s_%s.jpg" % (base_filename, save_count)
    img.save(f)
    print "saved image %s" % f
    save_count += 1

def _init_particle_filter(img):
    '''
    Internal wrapper to particle filter initializer.
    '''
    global particle_filter
    if not particle_filter:
        particle_filter = external_init_particle_filter(img)


def is_basket_middle(img):
    '''
    Single entry function returning True/False if basket is in the middle of
    the screen
    '''
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
