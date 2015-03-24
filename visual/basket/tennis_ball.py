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


def _get_tennis_blobs(img):
    '''
    Used for detecting tennis ball via RGB values.

    CURRENTLY NOT BEING USED.
    '''
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
    '''
    Picks out largest blob out of the bunch, used by RGB 
    detection approach.

    CURRENTLY NOT BEING USED.
    '''
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
    '''
    Filters certain colors out of an image for better 
    recognition, used by RGB approach.

    CURRENTLY NOT BEING USED.
    '''
    #removes less-red reds if they don't have enough blue
    removal_mask = img.createBinaryMask(color1=(30,0,0), 
            color2=(70,255,100)).invert()
    img = img.applyBinaryMask(removal_mask)

    # completely remove the red color
    (r,g,b) = img.splitChannels()
    img = img.mergeChannels(r=None, g=g, b=b)

    return img

def _check_point_feasibility(blob, img):
    '''
    Checks feasibility of point/blob given based on the history
    stored in the global PRQuadtree. This was the pre particle 
    filter apporach.

    CURRENTLY NOT BEING USED.
    '''
    global pr_tree, pr_tree_size

    if blob is None:
        return False 

    middle_pt = Point(img.width / 2, img.height / 2)

    # initialize tree
    if pr_tree is None:
        # NOTE: square crops a bit of image but should be ok
        box = Box(middle_pt, img.height / 2)
        pr_tree = PRQuadTree(box)

    # get center of given blob
    centroid = blob.centroid()
    blob_x = centroid[0]
    blob_y = centroid[1]
    
    # throw in some data initially
    if pr_tree_size < 10:
        inserted = pr_tree.insert(blob_x, blob_y)
        if inserted:
            pr_tree_size += 1
        return inserted

    # check within 100 pixels
    box_rng = Box(Point(blob_x, blob_y), 100)
    rng_points = pr_tree.query_range(box_rng)

    #print "around: %s" % len(rng_points)
    #print "pts: %s" % (rng_points)
    # WARNING: size method is recursion heavy!
    #print "pr-size: %s" % PRQuadTree.size(pr_tree)

    inserted = False
    # point feasible
    if len(rng_points) > 1:
        inserted = pr_tree.insert(blob_x, blob_y)
        if inserted:
            pr_tree_size += 1

    # clear tree to minimize load
    if pr_tree_size > 100:
        pr_tree_size = 0
        pr_tree = None
    return inserted

def _init_particle_filter(img):
    '''
    Internal wrapper to particle filter initializer.
    '''
    global particle_filter
    if not particle_filter:
        particle_filter = external_init_particle_filter(img)

def _ball_image_hue_filter(img):
    '''
    Internal wrapper image hue filter.
    '''
    color = (185, 206, 111)
    return image_hue_filter(img, color)


# entry point for module, returns none is ball is 
# not in middle, otherwise returns image with circle 
# drawn around ball if found
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


def run():
    global particle_filter

    cam = Camera()
    disp = Display()

    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()
        org_img = img

        # close window with left click
        if disp.mouseLeft:
            break

        _init_particle_filter(img)
        img = _ball_image_hue_filter(img)
        blobs = get_hue_blobs(img)
        if blobs:
            particle_filter.iterate(blobs)
            best = get_best_blob(blobs, particle_filter)
            if best:
                rad = best.radius()
                centroid = best.centroid()
                # error buffer for drawing circle on img
                rad += 10 
                # draw circle on picture
                org_img.drawCircle(centroid, rad, (0,255,0), 2)
                if is_blob_in_middle_helper(img, best):
                    print "BALL IN MIDDLE!"

        org_img.save(disp)

run()
