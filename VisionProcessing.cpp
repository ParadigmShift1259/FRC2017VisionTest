/*
 * Vision.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: Developer
 */
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

Mat src;
Mat src_gray;
int thresh = 100;
int max_thresh = 255;

const Scalar lower = Scalar(100, 255, 255);
const Scalar upper = Scalar(255, 255, 255);

void processFrame(Mat* binaryframe);

int main() {

        VideoCapture videostream = VideoCapture(0); 


	if (!videostream.isOpened()) {
		cerr << "error openning the camera" << endl
	}
	while (videostream.isOpened()) {
		// read an image
		videostream.read(streamimage);
		// process the frame
		processFrame(&streamimage);
	}

	return 0;
}

void processFrame(Mat* streamimage) {
	// create mats for processing steps
	Mat hsvimage;
	Mat inrange;
	
	// create vectors for contour analysis
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	// convert to hsv color space
	cvtColor(*streamimage, hsvimage, CV_BGR2HSV);
	
	// threshold the image into a binary image
	inRange(hsvimage, lower, upper, inrange);

	// Find contours
	findContours(*binaryframe, contours, hierarchy, CV_RETR_LIST,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// create vectors to hold the image moments, and the 2 largest contour moments
	vector<Moments> mu(contours.size());
	vector<Moments> biggest(2);
	Moments zero = Moments();
	zero.m00 = 0;
	biggest[0] = zero;
	biggest[1] = zero;
	// go thorugh all the contours and save the moments of the two largest
	for (unsigned int i = 0; i < contours.size(); i++) {
		mu[i] = moments(contours[i], false);
		if (mu[i].m00 > biggest[0].m00) {
			biggest[1] = biggest[0];
			biggest[0] = mu[i];
		} else if (mu[i].m00 > biggest[1].m00) {
			biggest[1] = mu[i];
		}
	}
	// calculate the mass ceters of the two largest contours
	Point2d p0;
	Point2d p1;
	if (biggest[1].m00 > 300) {
		p0 = Point2d(biggest[0].m10/biggest[0].m00, biggest[0].m01/biggest[0].m00);
		p1 = Point2d(biggest[1].m10/biggest[1].m00, biggest[1].m01/biggest[1].m00);
	}
	// display the points to the console (x0 is larger)
	cout << "x0 " << p0.x << "y0 " << p0.y << endl;
	cout << "x1 " << p1.x << "y1 " << p1.y << endl;
}

