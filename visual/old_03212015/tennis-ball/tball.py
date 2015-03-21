'''
Simple detection of ball using SimpleCV (much easier than OpenCV). The run method
identifies a tennis ball in the camera stream image. '_is_ball_in_middle' function
can be used to determine whether a ball is horizontally centered based on a specified
threshold.

-Pawel Szczurko
'''

from SimpleCV import *
from time import sleep
import sys

# function for experimental testing of acceptable
# average color ranges
def _determine_range(avg_color):
    global MIN_R, MIN_G, MIN_B, MAX_R, MAX_G, MAX_B

    cur_r = avg_color[0]
    cur_g = avg_color[1]
    cur_b = avg_color[2]

    if cur_r < MIN_R:
        MIN_R = cur_r
    if cur_r > MAX_R:
        MAX_R = cur_r

    if cur_g < MIN_G:
        MIN_G = cur_g
    if cur_g > MAX_G:
        MAX_G = cur_g

    if cur_b < MIN_B:
        MIN_B = cur_b
    if cur_b > MAX_B:
        MAX_B = cur_b
    print "MAX: (%i, %i, %i), MIN(%i, %i, %i)" % (MAX_R, MAX_G, MAX_B,
            MIN_R, MIN_G, MIN_B)

def _is_ball_in_middle(img_middle, blob_x):
    # experimental threshold for center
    THRESHOLD = 50

    if (img_middle - THRESHOLD < blob_x 
            and blob_x < img_middle + THRESHOLD):
        return True
    return False

def _get_tennis_blobs(img):
    # accepted color range (when no color removal)
    #start_color = (123,132,51)
    #end_color = (203,232,106)

    # with red removed
    start_color = (0, 166, 124)
    end_color = (255, 220, 147)

    
    # create binary mask based on color ranges
    mask = img.createBinaryMask(color1=start_color,color2=end_color)

    # find binary blobs in the image
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs

def _get_largest_blob(blobs):
    # find the largest blob
    largest_blob = None
    if blobs is not None:
        for b in blobs:
            if largest_blob is None:
                largest_blob = b
            elif b.area() > largest_blob.area():
                largest_blob = b
    return largest_blob

def _image_color_filter(img):
    #removes less-red reds if they don't have enough blue
    removal_mask = img.createBinaryMask(color1=(30,0,0), 
            color2=(70,255,100)).invert()
    img = img.applyBinaryMask(removal_mask)

    # completely remove the red color
    (r,g,b) = img.splitChannels()
    img = img.mergeChannels(r=None, g=g, b=b)

    return img

def run():
    cam = Camera()
    disp = Display()

    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        img = _image_color_filter(img)

        # close window with left click
        if disp.mouseLeft:
            break
        blobs = _get_tennis_blobs(img)
        largest_blob = _get_largest_blob(blobs)
    
        if(largest_blob is not None):
            rad = largest_blob.radius()
            centroid = largest_blob.centroid()

            print _is_ball_in_middle(img.width / 2, centroid[0])  

            # error buffer for drawing circle on img
            rad += 10 
            # draw circle on picture
            img.drawCircle(centroid, rad, (0,255,0), 2)

        img.save(disp)

run()
