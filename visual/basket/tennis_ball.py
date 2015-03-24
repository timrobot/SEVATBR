'''
Simple detection of ball using SimpleCV (much easier than OpenCV). The run method
identifies a tennis ball in the camera stream image. 'is_ball_middle' function
can be used to determine whether a ball is horizontally centered based on a specified
threshold.

-Pawel Szczurko
'''

from SimpleCV import *
from time import sleep
from prquadtree import *
from particlefilter import ParticleFilter
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
    global particle_filter
    if particle_filter is None:
        middle_pt = Point(img.width / 2, img.height / 2)
        # NOTE: square crops a bit of image but should be ok
        box = Box(middle_pt, img.height / 2)
        particle_filter = ParticleFilter(box)

def basket_image_hue_filter(img):
    return img.hueDistance((185,206,111), minsaturation=110, minvalue=130)

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

def _is_blob_in_middle_helper(img, blob):
    img_middle = img.width/2
    blob_x = blob.centroid()[0]
    # experimental threshold for center
    THRESHOLD = 50
    if (img_middle - THRESHOLD < blob_x and blob_x < img_middle + THRESHOLD):
        return True
    return False


# entry point for module, returns none is ball is 
# not in middle, otherwise returns image with circle 
# drawn around ball if found
def is_ball_middle(img):

    _init_particle_filter(img)
    img = basket_image_hue_filter(img)
    blobs = _get_basket_hue_blobs(img)
    if blobs:
        particle_filter.iterate(blobs)
        best = _get_best_blob(blobs)
        return _is_blob_in_middle_helper(img, best)
    return False


def run():
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
        img = basket_image_hue_filter(img)
        blobs = _get_basket_hue_blobs(img)
        if blobs:
            particle_filter.iterate(blobs)
            best = _get_best_blob(blobs)
            if best:
                rad = best.radius()
                centroid = best.centroid()
                # error buffer for drawing circle on img
                rad += 10 
                # draw circle on picture
                org_img.drawCircle(centroid, rad, (0,255,0), 2)
                if _is_blob_in_middle_helper(img, best):
                    print "BALL IN MIDDLE!"

        org_img.save(disp)

run()
