from SimpleCV import *
from time import sleep
import sys
from datetime import datetime
import numpy as np
import experiment

#silly globals
save_count = 1
base_filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

def _basket_image_filter(img):
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

def _get_basket_blobs(img):
    # accepted color range
    start_color = (0,0,35)
    end_color = (255,255,255)

    # create binary mask based on color ranges
    mask = img.createBinaryMask(color1=start_color,color2=end_color)

    # find binary blobs in the image
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs

def _get_best_blob(blobs):
    def c_diff(c1, c2):
        return sum([abs(x-y) for x,y in zip(c1,c2)]) #sums the abs diff

    if blobs is None:
        return None
    # find the largest blob which has closest mean color to target color
    target_color = (20, 60, 100)
    largest_score = False
    best_blob = None
    for b in blobs:
        score = b.area() - c_diff(b.meanColor(), target_color)
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


def run():
    cam = Camera()
    disp = Display()

    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        img = _basket_image_filter(img)
        blobs = _get_basket_blobs(img)
        if blobs:
            blobs.show()
            for b in blobs:
                if b.isRectangle(.2):
                    b.drawRect(color=Color.RED)
            best = _get_best_blob(blobs)
            best.drawRect(color=Color.GREEN)
        img.save(disp)
        if disp.mouseLeft:
            break
        if disp.mouseRight:
            _save_image(img)


run()
#experiment.experiment()