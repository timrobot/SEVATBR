#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

vector<Rect> faces;
VideoCapture cap;

void setup_params(DetectionBasedTracker::Parameters &param) {
  param.maxObjectSize = 400;
  param.maxTrackLifetime = 20;
  param.minDetectionPeriod = 7;
  param.minNeighbors = 3;
  param.minObjectSize = 20;
  param.scaleFactor = 1.1;
}

void fdinit() {
  cap.open(0);
  cv::Rect_<int> face_i;
  cv::namedWindow("tracker", cv::WINDOW_NORMAL);
  cv::setWindowProperty("tracker", CV_WIND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
}

void main(int argc, char *argv[]) {

}
