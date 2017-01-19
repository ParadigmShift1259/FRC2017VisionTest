/*
 * Vision.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: Developer
 */
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../raspicam-0.1.3/src/raspicam.h"
#include "../raspicam-0.1.3/src/raspicam_cv.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;
using namespace raspicam;

Mat src;
Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

void contouringstuffs(Mat inrange);

int main() {
	RaspiCam_Cv Camera; //Camera object
	Mat streamimage;
	Mat hsvimage;
	Mat inrange;
	Camera.open();
	Camera.grab();
	for(int x=0; x < 1; x++) {
		Camera.retrieve(streamimage);
		if(Camera.isOpened()){
			cout<<"Opened Camera"<< endl;
			Scalar lower = Scalar(0, 0, 100);
			Scalar upper = Scalar(180, 255, 255);
			cvtColor(streamimage, hsvimage, CV_BGR2HSV);
			inRange(hsvimage, lower, upper, inrange); 
			cv::imwrite("rapspicam1.jpg", streamimage);
			cv::imwrite("raspicam_pic.jpg",inrange);
		} else {
			cout << "Failed to open camera" << endl;
			break;
		}
	}
	
	contouringstuffs(inrange);
	Camera.release();
	return 0;
}

void contouringstuffs(Mat inrange)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(inrange, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	 Mat drawing = Mat::zeros( inrange.size(), CV_8UC3 );

  	for( size_t i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar(rng.uniform(0, 255),rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i , color, 2, 8, hierarchy, 0);
     } 
	cv::imwrite("contourImage.jpg", drawing);
	
}


