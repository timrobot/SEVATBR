// Basic video camera image caputre through OpenCV
#include <stdio.h>
#include <cv.h>
#include <highgui.h>

void cvOpen(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
        cvErode (src, dst, element, 1);
            cvDilate(src, dst, element, 1);
}

void cvClose(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
        cvDilate(src, dst, element, 1);
            cvErode (src, dst, element, 1);
}

int main()
{
    char key;
    cvNamedWindow("Camera_Output", 1); //Create window
    CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY); //Capture using any camera connected to your system
    while(1){ //Create infinte loop for live streaming

        IplImage* img = cvQueryFrame(capture); //Create image frames from Capture
/*        using this to figure out the segfault. I'm presuming it is due to memoryleak of sorts
 *        cvShowImage("img", img);
        key = cvWaitKey(10); //Capture Keyboard stroke
        if ((char)key == 27){
            break; //If you hit ESC key loop will break.
        }
        continue;*/
        CvSize size = cvGetSize(img);
        /* convert to hsv */
        IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
        cvCvtColor(img, hsv, CV_BGR2HSV);

        /* create color mask */
        CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
        /* TODO: fine tune this range */
        cvInRangeS(hsv, cvScalar(0.11*256, 0.60*256, 0.20*256, 0),
                cvScalar(0.14*256, 1.00*256, 1.00*256, 0), mask);
//        cvInRangeS(hsv, cvScalar(49, 124, 235, 0),
  //              cvScalar(0.14*256, 1.00*256, 1.00*256, 0), mask);

        cvReleaseImage(&hsv);
        /* take out the noise out of b-w image*/
        IplConvKernel *se21 = cvCreateStructuringElementEx(21, 21, 10, 10, CV_SHAPE_RECT, NULL);
        IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5,  5,  CV_SHAPE_RECT, NULL);
        cvClose(mask, mask, se21); 
        cvOpen(mask, mask, se11); 
        cvReleaseStructuringElement(&se21);
        cvReleaseStructuringElement(&se11);


        /* Copy mask into a grayscale image */
        IplImage *hough_in = cvCreateImage(size, 8, 1);
        cvCopy(mask, hough_in, NULL);
        cvSmooth(hough_in, hough_in, CV_GAUSSIAN, 15, 15, 0, 0);

        /* Run the Hough function */
        CvMemStorage *storage = cvCreateMemStorage(0);
        CvSeq *circles = cvHoughCircles(hough_in, storage,
                CV_HOUGH_GRADIENT, 4, size.height/10, 100, 40, 0, 0);
        cvReleaseMemStorage(&storage);

        /* draw circles */
        int i;
        for (i = 0; i < circles->total; i++) {
            float *p = (float*)cvGetSeqElem(circles, i);
            CvPoint center = cvPoint(cvRound(p[0]),cvRound(p[1]));
            CvScalar val = cvGet2D(mask, center.y, center.x);
            if (val.val[0] < 1) continue;
            cvCircle(img,  center, 3,CV_RGB(0,255,0), -1, CV_AA, 0);
            cvCircle(img,  center, cvRound(p[2]), CV_RGB(255,0,0),  3, CV_AA, 0);
            cvCircle(mask, center, 3,CV_RGB(0,255,0), -1, CV_AA, 0);
            cvCircle(mask, center, cvRound(p[2]), CV_RGB(255,0,0),  3, CV_AA, 0);
        }

        cvShowImage("orig image", img);
        cvShowImage("Camera_Output", mask);

        key = cvWaitKey(10); //Capture Keyboard stroke
        if ((char)key == 27){
            break; //If you hit ESC key loop will break.
        }
    }
    cvReleaseCapture(&capture); //Release capture.
    cvDestroyWindow("Camera_Output"); //Destroy Window
    return 0;
} 
