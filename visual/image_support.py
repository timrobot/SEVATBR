from SimpleCV import *
from prquadtree import *
from particlefilter import ParticleFilter

## Initializes particle filter.
#
#    @param img SimpleCV.Image captured image
#    @return  A ParticleFilter object
#    
def external_init_particle_filter(img):
    middle_pt = Point(img.width / 2, img.height / 2)
    # NOTE: square crops a bit of image but should be ok
    box = Box(middle_pt, img.height / 2)
    particle_filter = ParticleFilter(box)
    return particle_filter

## Converts given image to HSV based on the given color.
#
# @param img SimpleCV.Image captured image
# @param color tuple of RGB values of singe 'H' value of HSV
# @return HSV converted image 
#
def image_hue_filter(img, ball=True):
    global bcolor
    if ball:
        # good tested values
        return img.hueDistance(32, minsaturation=72, minvalue=120)
    # good tested values
    return img.hueDistance(105, minsaturation=80, minvalue=75)

## Gets basket blobs after hue distance filtering.
#
# @param img SimpleCV.Image captured image.
# @return Set of 'black' potential blobs.
#
def get_hue_blobs(img):
    # accepted color range
    start_color = (0,0,0)
    end_color = (20,20,20)

    # create binary mask based on color ranges
    mask = img.createBinaryMask(color1=start_color,color2=end_color)

    # find binary blobs in the image
    blobs = img.findBlobsFromMask(mask, appx_level=10, minsize=20)
    return blobs

## Returns the best blob out of the provided set and particle filter.
#
# @param blobs: list of potential HSV blobs
# @param particle_filter: initialized ParticleFilter object 
# @return The largest blob found or None.
def get_best_blob(blobs, particle_filter):
    def c_diff(c1, c2):
        return sum([abs(x-y) for x,y in zip(c1,c2)]) #sums the abs diff

    if blobs is None:
        return None
    # find the largest blob which has closest mean color to target color
    largest_score = False
    best_blob = None
    for b in blobs:
        score = b.area()
        #take particle filter score into account
        score += particle_filter.score(b)
        if largest_score is False or score > largest_score:
            best_blob = b
            largest_score = score
    return best_blob

## Determines whether the given blob is in ceter of image.
#
# @param img SimpleCV.Image caputed image
# @param blob SimpleCV.Blob Blob object
# @return True if blob in middle of image, false otherwise.
def is_blob_in_middle_helper(img, blob):
    img_middle = img.width/2
    blob_x = blob.centroid()[0]
    # experimental threshold for center
    THRESHOLD = 50
    if (img_middle - THRESHOLD < blob_x and blob_x < img_middle + THRESHOLD):
        return True
    return False

