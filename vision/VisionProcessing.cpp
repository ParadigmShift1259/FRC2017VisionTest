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
#include <networktables/NetworkTable.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <thread>

using namespace cv;
using namespace std;
using namespace raspicam;

shared_ptr<NetworkTable> netTable;
Scalar lower = Scalar(0, 0, 80);
Scalar upper = Scalar(180, 255, 255);
Mat frame;
RaspiCam_Cv Camera; //Camera object
Mat streamimage;
Mat hsvimage;
Mat inrange;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
Mat drawing;

RNG rng(12345);

void grabframe();
//draws 2 largest contours and returns said two largest contours in a vector of contours
vector<vector<Point> > contouringstuffs();
//used with sort in contouringstuffs
bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );
//gets mentioned point from a specific contour
cv::Point getLeftmost(vector<cv::Point> newContour);
cv::Point getRightmost(vector<cv::Point> newContour);
cv::Point getTopmost(vector<cv::Point> newContour);
cv::Point getBotmost(vector<cv::Point> newContour);
//gets the x value between two contours and then returns the amount of pixels that the bot is off
int getCenteredX(vector<vector<Point> > twoContours);

int main() {
	Camera.open();
	for(int i = 0; i < 20; i++)
	{
		Camera.grab();
	}
	Camera.retrieve(frame);
	cv::imwrite("firstframe.jpg",frame);
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress("roboRIO-1259-FRC.local");
	netTable = NetworkTable::GetTable("OpenCV");
	if(Camera.isOpened()){
		thread first (grabframe);
		cout<<"Opened Camera"<< endl;
		while (frame.empty());
		thread second (contouringstuffs);
		first.join();
		second.join();
	} else {
		cout << "Failed to open camera" << endl;
	}

	Camera.release();
	return 0;
}

void grabframe()
{
	while(true){
		Camera.grab();
        	Camera.retrieve(frame);
	}
}


vector<vector<Point> > contouringstuffs()
{
	while(true){
		cvtColor(frame, hsvimage, CV_BGR2HSV);
		inRange(hsvimage, lower, upper, inrange);
		cv::imwrite("rapspicam1.jpg", frame);
		cv::imwrite("raspicam_pic.jpg",inrange);

		findContours(inrange, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

		drawing = Mat::zeros( inrange.size(), CV_8UC3 );
		//Returns array sorted smallest to biggest
		std::sort(contours.begin(), contours.end(), compareContourAreas);

	        Scalar color = Scalar(rng.uniform(0, 255),rng.uniform(0,255), rng.uniform(0,255) );
       		drawContours( drawing, contours,(contours.size()-1), color, 2, 8, hierarchy, 0);
		drawContours( drawing, contours, (contours.size()-2), color, 2, 8, hierarchy, 0);

		cv::imwrite("contourImage.jpg", drawing);

		if(!(contours.size() < 2))
		{
			vector<vector<Point> > finalcontours;
			finalcontours.resize(2);
			//do identification of target here... Currently grabs the two largest targets that pass thresholding
			finalcontours.at(0) = contours.at(contours.size()-1);
			finalcontours.at(1) = contours.at(contours.size()-2);

			cout << (getCenteredX(finalcontours) - (hsvimage.size().width/2)) << endl;
		        netTable->PutNumber("area",(getCenteredX(contours) - (hsvimage.size().width/2)));
		        netTable ->PutNumber("notOld", 1);
		}
		else
		{
		cout << "Did not find two contours" << endl;
	        netTable->PutNumber("notOld", 0);
		}
	}
}

int getCenteredX(vector<vector<Point> > twoContours)
{
	int leftContourRightx = getLeftmost(twoContours.at(0)).x;
	int rightContourLeftx = getLeftmost(twoContours.at(1)).x;
	if(leftContourRightx > rightContourLeftx)
	{
		std::iter_swap(twoContours.begin(), twoContours.begin()+1);
		rightContourLeftx = leftContourRightx;
		leftContourRightx = getRightmost(twoContours.at(0)).x;
	}
	else
	{
	leftContourRightx = getRightmost(twoContours.at(0)).x;
	}

	return (leftContourRightx + ((rightContourLeftx - leftContourRightx)/2));
}


//Used with sort
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}

Point getLeftmost(vector<cv::Point> contour)
{
return *min_element(contour.begin(), contour.end(), 
                      [](const Point& lhs, const Point& rhs) {
                          return lhs.x < rhs.x;
                  }); 
}

Point getRightmost(vector<cv::Point> contour)
{
return *max_element(contour.begin(), contour.end(),
                      [](const Point& lhs, const Point& rhs) {
                          return lhs.x < rhs.x;
		});
}

Point getTopmost(vector<cv::Point> contour)
{
return *min_element(contour.begin(), contour.end(), 
                      [](const Point& lhs, const Point& rhs) {
                          return lhs.y < rhs.y;
		});
}

Point getBotmost(vector<cv::Point> contour)
{
return *max_element(contour.begin(), contour.end(),
                      [](const Point& lhs, const Point& rhs) {
                          return lhs.y < rhs.y;
                  });
}
