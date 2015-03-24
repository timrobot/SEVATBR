from SimpleCV import *
from prquadtree import *
from particlefilter import ParticleFilter

bcolor = 0


def external_init_particle_filter(img):
    '''
    Initializes particle filter.

    Args:
        img: SimpleCV captured image

    Returns:
        A ParticleFilter object
    '''
    middle_pt = Point(img.width / 2, img.height / 2)
    # NOTE: square crops a bit of image but should be ok
    box = Box(middle_pt, img.height / 2)
    particle_filter = ParticleFilter(box)
    return particle_filter

def image_hue_filter(img, ball=True):
    global bcolor
    '''
    Converts given image to HSV based on the given color.

    Args:
        img: SimpleCV captured image
        color: tuple of RGB values of singe 'H' value of HSV

    Returns:
        HSV converted image
    '''
    if ball:
        return img.hueDistance(45, minsaturation=49, minvalue=69)
    bcolor += 1
    print "Bitch: %s" % bcolor
    return img.hueDistance(105, minsaturation=80, minvalue=72)


def get_hue_blobs(img):
    '''
    Gets basket blobs after hue distance filtering.

    Args:
        img: SimpleCV captured image.

    Returns:
        Set of 'black' potential blobs.
    '''
    # accepted color range
    start_color = (0,0,0)
    end_color = (20,20,20)

    # create binary mask based on color ranges
    mask = img.createBinaryMask(color1=start_color,color2=end_color)

    # find binary blobs in the image
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs

def get_best_blob(blobs, particle_filter):
    '''
    Returns the best blob out of the provided set and particle filter.

    Args:
        blobs: list of potential HSV blobs
        particle_filter: initialized ParticleFilter object 

    Returns:
        The largest blob found or None.
    '''
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

def is_blob_in_middle_helper(img, blob):
    '''
    Determines whether the given blob is in ceter of image.

    Args:
        img: SimpleCV caputed image
        blob: SimpleCV Blob object

    Returns:
        True if blob in middle of image, false otherwise.
    '''
    img_middle = img.width/2
    blob_x = blob.centroid()[0]
    # experimental threshold for center
    THRESHOLD = 50
    if (img_middle - THRESHOLD < blob_x and blob_x < img_middle + THRESHOLD):
        return True
    return False



