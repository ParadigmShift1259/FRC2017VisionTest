/*
 * Vision.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: Developer
 */
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../raspicam-0.1.3/src/raspicam.h"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

Mat src;
Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

void thresh_callback(int, void*);

int main() {
	raspicam::RaspiCam Camera(); //Camera object
	Camera.open();
	Camera.grab();
	unsigned char *data=new unsigned char[ Camera.getImageTypeSize (raspicam::RASPICAM_FORMAT_BGR )];
	Camera.retrieve (data,raspicam::RASPICAM_FORMAT_BGR);
	std::ofstream outFilef("raspicam_image.ppm",std::ios::binary);
	outFile<<"P6\n"<<Camera.getWidth() << " " <<Camera.getHeight() <<" 255/n";
	outFile.write( (char*) data, Camera.getImageTypeSize (raspicam::RASPICAM_FORMAT_BGR));

	VideoCapture videostream(0);
	videostream.open(0);
	while(1) {
		Mat streamimage;
		Mat hsvimage;
		Mat inrange;
		bool captured = videostream.read(streamimage);
		if (videostream.isOpened()) {
			cout<<"Opened Camera"<< endl;
			Scalar lower = Scalar(100, 255, 255);
			Scalar upper = Scalar(255, 255, 255);
			cvtColor(streamimage, hsvimage, CV_BGR2HSV);
			inRange(hsvimage, lower, upper, inrange);
		} else {
			cout << "Failed to opfdsfdsaen camera" << endl;
			break;
		}
	}
	return 0;
}

void thresh_callback(int, void*) {
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

//Detect edges using canny
	Canny(src_gray, canny_output, thresh, thresh * 2, 3);

//Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

//Get moments
	vector<Moments> mu(contours.size());
	vector<Moments> biggest(2);
	Moments zero = Moments();
	zero.m00 = 0;
	biggest[0] = zero;
	biggest[1] = zero;
	for (unsigned int i = 0; i < contours.size(); i++) {
		mu[i] = moments(contours[i], false);
		if (mu[i].m00 > biggest[0].m00) {
			biggest[1] = biggest[0];
			biggest[0] = mu[i];
		} else if (mu[i].m00 > biggest[1].m00) {
			biggest[1] = mu[i];
		}
	}
	Point2d p0;
	Point2d p1;
	if (biggest[1].m00 > 300) {
		p0 = Point2d(biggest[0].m10/biggest[0].m00, biggest[0].m01/biggest[0].m00);
		p1 = Point2d(biggest[1].m10/biggest[1].m00, biggest[1].m01/biggest[1].m00);
	}
	cout << "x0 " << p0.x << "y0 " << p0.y << endl;
	cout << "x1 " << p1.x << "y1 " << p1.y << endl;
}

