from SimpleCV import *
from time import sleep
import sys
from datetime import datetime
import numpy as np
import experiment
from prquadtree import *
from particlefilter import ParticleFilter

#silly globals
particle_filter = None
image_half_size = -1
save_count = 1
base_filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

def basket_image_filter(img):
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

def basket_image_hue_filter(img):
    return img.hueDistance(280, minsaturation=110, minvalue=130)

def _get_basket_blobs(img):
    '''
    Gets basket blobs with RGB range
    '''
    # accepted color range
    start_color = (0,0,35)
    end_color = (255,255,255)

    # create binary mask based on color ranges
    mask = img.createBinaryMask(color1=start_color,color2=end_color)

    # find binary blobs in the image
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs

def _get_basket_hue_blobs(img):
    '''
    Gets basket blobs after hue distance filtering
    '''
    # accepted color range
    start_color = (0,0,0)
    end_color = (20,20,20)

    # create binary mask based on color ranges
    mask = img.createBinaryMask(color1=start_color,color2=end_color)

    # find binary blobs in the image
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs

def _get_best_blob(blobs):
    global particle_filter
    def c_diff(c1, c2):
        return sum([abs(x-y) for x,y in zip(c1,c2)]) #sums the abs diff

    if blobs is None:
        return None
    # find the largest blob which has closest mean color to target color
    #target_color = (20, 60, 100)
    largest_score = False
    best_blob = None
    for b in blobs:
        score = b.area() #- c_diff(b.meanColor(), target_color)
        #take particle filter score into account
        score += particle_filter.score(b)
        if largest_score is False or score > largest_score:
            best_blob = b
            largest_score = score
    return best_blob

def _save_image(img):
    global save_count, base_filename
    f = "cam_capture_%s_%s.jpg" % (base_filename, save_count)
    img.save(f)
    print "saved image %s" % f
    save_count += 1

def _init_particle_filter(img):
    global particle_filter
    if particle_filter is None:
        middle_pt = Point(img.width / 2, img.height / 2)
        # NOTE: square crops a bit of image but should be ok
        box = Box(middle_pt, img.height / 2)
        particle_filter = ParticleFilter(box)

def _is_blob_in_middle_helper(img, blob):
    img_middle = img.width/2
    blob_x = blob.centroid()[0]
    # experimental threshold for center
    THRESHOLD = 50
    if (img_middle - THRESHOLD < blob_x and blob_x < img_middle + THRESHOLD):
        return True
    return False

def is_basket_middle(img):
    '''
    Single entry function returning True/False if basket is in the middle of
    the screen
    '''
    _init_particle_filter(img)
    img = basket_image_hue_filter(img)
    blobs = _get_basket_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = _get_best_blob(blobs)
        return _is_blob_in_middle_helper(img, best)
    return False

def run_middle():
    '''
    Runs continuously and prints if the best detected blob is in the middle
    '''
    def middle_callback(img, best):
        if _is_blob_in_middle_helper(img, best):
            print "Basket in middle"
    run(middle_callback)

def run(bestBlobCallback=False):
    cam = Camera()
    disp = Display()
    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        img = basket_image_hue_filter(img)
        _init_particle_filter(img)
        blobs = _get_basket_hue_blobs(img)
        if blobs:
            particle_filter.iterate(blobs)
            #blobs.show()
            for b in blobs:
                if b.isRectangle(.2):
                    b.drawRect(color=Color.RED)
            best = _get_best_blob(blobs)
            best.drawRect(color=Color.GREEN)
            if bestBlobCallback:
                bestBlobCallback(img, best)
        img.save(disp)
        if disp.mouseLeft:
            break
        if disp.mouseRight:
            _save_image(img)
