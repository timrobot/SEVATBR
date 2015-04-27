#include <math.h>
#include <stdio.h>
#include "kinect.hpp"

using namespace cv;
using namespace std;

/* This file accesses the Kinect Device and gets its video and depth frames. If a depth frame is deteced, a new distance frame is created as well */

KinectDevice::KinectDevice(freenect_context *_ctx, int _index) :
    Freenect::FreenectDevice(_ctx, _index),
    depth_buffer(FREENECT_DEPTH_11BIT),
    video_buffer(FREENECT_VIDEO_RGB),
    gamma_buffer(2048),
    new_depth_frame(false),
    new_video_frame(false),
    depthMat(Size(640, 480), CV_16UC1),
    videoMat(Size(640, 480), CV_8UC3),
    new_bgr_frame(false),
    new_meters_frame(false),
    raw2bgrMat(Size(640, 480), CV_8UC3),
    distanceMat(Size(640, 480), sizeof(double)) {
  int i;
  for (i = 0; i < 2048; i++) {
    float v = i / 2048.0;
    v = pow(v, 3) * 6;
    gamma_buffer[i] = v * 6 * 256;
  }
  pthread_mutex_init(&depth_lock, NULL);
  pthread_mutex_init(&video_lock, NULL);
}

KinectDevice::~KinectDevice() {
  pthread_mutex_destroy(&depth_lock);
  pthread_mutex_destroy(&video_lock);
}

void KinectDevice::DepthCallback(void *data, uint32_t timestamp) {
  pthread_mutex_lock(&depth_lock);
  depthMat.data = (uint8_t *)(uint16_t *)data;
  new_depth_frame = true;
  new_distance_frame = true;
  pthread_mutex_unlock(&depth_lock);
}

void KinectDevice::VideoCallback(void *data, uint32_t timestamp) {
  pthread_mutex_lock(&video_lock);
  videoMat.data = (uint8_t *)data;
  new_video_frame = true;
  pthread_mutex_unlock(&video_lock);
}

bool KinectDevice::getDepth(Mat& output) {
  pthread_mutex_lock(&depth_lock);
  if (new_depth_frame) {
    depthMat.copyTo(output);
    new_depth_frame = false;
    pthread_mutex_unlock(&depth_lock);
    return true;
  } else {
    depthMat.copyTo(output);
    pthread_mutex_unlock(&depth_lock);
    return false;
  }
}

bool KinectDevice::getVideo(Mat& output) {
  pthread_mutex_lock(&video_lock);
  if (new_video_frame) {
    cvtColor(videoMat, output, CV_RGB2BGR);
    new_video_frame = false;
    pthread_mutex_unlock(&video_lock);
    return true;
  } else {
    cvtColor(videoMat, output, CV_RGB2BGR);
    pthread_mutex_unlock(&video_lock);
    return false;
  }
}

uint16_t sigfun(double x) {
  return (uint16_t)(x * 150.0);
}

bool KinectDevice::getDistanceMat(void) {
  if (new_depth_frame) {
    getDepth(depthMat);
  }
  if (new_meters_frame) {
    for (int y = 0; y < depthMat.rows; y++)
      for (int x = 0; x < depthMat.cols; x++)
        distanceMat(y, x) = raw2meters(depthMat.at<uint16_t>(y, x));
    distanceMat.copyTo(output);
    new_meters_frame = false;
    return true;
  } else {
    distanceMat.copyTo(output);
    return false;
  }
}

double KinectDevice::raw2meters(uint16_t raw) {
  // stephane maganate
  return (0.1236 * tan((double)raw / 2842.5 + 1.1863));
}
