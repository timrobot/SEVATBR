'''
Simple detection of ball using SimpleCV (much easier than OpenCV). The run method
identifies a tennis ball in the camera stream image. 'is_ball_middle' function
can be used to determine whether a ball is horizontally centered based on a specified
threshold.

-Pawel Szczurko
'''

from SimpleCV import *
from time import sleep
from particlefilter import ParticleFilter
from image_support import *
import sys

# global particle filter
# error point elimination
particle_filter = None
mode = "ball" # basket|ball

## Internal wrapper to particle filter initializer.
#
# @param img SimpleCV.Image
def _init_particle_filter(img):
    global particle_filter
    if not particle_filter:
        particle_filter = external_init_particle_filter(img)

## Internal wrapper image hue filter. 
#
# @param img SimpleCV.Image 
# @return img SimpleCV.Image converted to HSV
def _ball_image_hue_filter(img):
    color = (185, 206, 111)
    return image_hue_filter(img, True)

## Entry point for module which determines whether tennis ball is in the middle of the image.
#
# @param img SimpleCV.Image 
# @return boolean. True if tennis ball is in middle, false otherwise.
def is_ball_middle(img):
    global particle_filter

    _init_particle_filter(img)
    img = _ball_image_hue_filter(img)
    blobs = get_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = get_best_blob(blobs, particle_filter)
        return is_blob_in_middle_helper(img, best)
    return False

##
#Internal wrapper image hue filter.
#param img SimpleCV.Image the image to apply the hue filter to
#
def _basket_image_hue_filter(img):
    color = 280
    return image_hue_filter(img, False)
##
#Saves an image to the current directory
#@param img SimpleCV.Image the image to save
#
def _save_image(img):
    global save_count, base_filename
    f = "cam_capture_%s_%s.jpg" % (base_filename, save_count)
    img.save(f)
    print "saved image %s" % f
    save_count += 1

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

##
# Runs continuously and prints if the best detected blob is in the middle
#
def run_middle():
    def middle_callback(img, best):
        if is_blob_in_middle_helper(img, best):
            print "Basket in middle"
    run(middle_callback)



## Continuously captures image from computer camera and feeds it to the is_ball_middle method to detect whether tennis ball is in the middle of the screen.
def run():
    global particle_filter
    global mode
    cam = Camera()
    disp = Display()

    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        org_img = img

        # close window with left click
        if disp.mouseLeft:
            break

        if mode == "ball":        
            img = _ball_image_hue_filter(img)
            blobs = get_hue_blobs(img)
            if blobs:
                best = get_best_blob(blobs, particle_filter)
                if best:
                    rad = best.radius()
                    centroid = best.centroid()
                    print "Location: (%s, %s)" % (centroid[0], centroid[1])
                    # error buffer for drawing circle on img
                    #rad += 10 
                    # draw circle on picture
                    if is_blob_in_middle_helper(img, best):
                        org_img.drawCircle(centroid, rad, (0,255,0), 2)
                        print "BALL IN MIDDLE!"
            org_img.save(disp)
        elif mode == "basket":
            sleep(.05)
            img = cam.getImage()
            img = _basket_image_hue_filter(img)
            #_init_particle_filter(img)
            blobs = get_hue_blobs(img)
            if blobs:
                #blobs.show()
                best = get_best_blob(blobs, particle_filter)
                if is_blob_in_middle_helper(img, best):
                    print "About %s inches away" % (880000.0 / best.area())
                    best.drawRect(color=Color.BLUE, width=10)
        
            img.save(disp)
run()