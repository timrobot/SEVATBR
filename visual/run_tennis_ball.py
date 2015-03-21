'''
This imitates how the 'tennis_ball' module will actually be run. All that 
will happen is that an image will be passed ot the 'is_ball_middle' method
and it will return true or false.

-Pawel Szczurko
'''

from time import sleep
from SimpleCV import *
from tennis_ball import is_ball_middle

def run():
    cam = Camera()
    disp = Display()

    while disp.isNotDone():
        sleep(.05)
        img = cam.getImage()

        # close window with left click
        if disp.mouseLeft:
            break

        temp_img = is_ball_middle(img)
        if temp_img:
            print "MIDDLE!"
            img = temp_img

        img.save(disp)

run()
