#!/bin/bash
PORT=5000
gst-launch-0.10 -v udpsrc port=${PORT} ! \
    jpegdec ! \
    autovideosink
