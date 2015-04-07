#!/bin/bash

DEV=/dev/video0
URL=127.0.0.1
PORT=5000

gst-launch-0.10 v4l2src device=${DEV} ! 'video/x-raw-yuv,width=640,height=480,framerate=30/1' ! \
    queue ! ffmpegcolorspace ! jpegenc ! \
    udpsink host=${URL} port=${PORT}
