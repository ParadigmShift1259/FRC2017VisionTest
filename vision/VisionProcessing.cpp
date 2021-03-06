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
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <atomic>
#include <math.h>

using namespace cv;
using namespace std;
using namespace raspicam;

//const char* const  portname = "/dev/ttyAMA0";
int32_t globalX;
bool notOld;
double xOffDistance;
double xSpread;
double areaDifference;
//int fd;
bool foundValidPicture;
shared_ptr<NetworkTable> netTable;
const Scalar lower = Scalar(50, 80, 15); //hsv
//const Scalar lower = Scalar(15, 70, 0); //bgr
const Scalar upper = Scalar(70, 256, 256); //hsv
//const Scalar upper = Scalar(127, 256, 20); //bgr
RaspiCam_Cv Camera; //Camera object
Mat img;
Mat hsvimage;
Mat inrange;

vector<vector<Point>> contours;
//contour moment pair
typedef struct {
	vector<Point> *contour;
	Moments m;
} cmpair_t;

vector<cmpair_t> cmpairs;
int counter = 0;
vector<Vec4i> hierarchy;
Mat drawing;

const Scalar contourColor1 = Scalar(124, 252, 0);
const Scalar contourColor2 = Scalar(180, 105, 255);
//int set_interface_attribs(int fd, int speed, int parity);
//void set_blocking(int fd, int should_block);

void grabframe();
//draws 2 largest contours and returns said two largest contours in a vector of contours
void contouringstuffs();
//used with sort in contouringstuffs
bool compareContourAreas(cmpair_t cmpair1, cmpair_t cmpair2);
//gets mentioned point from a specific contour

// compares the opposite slopes of a quadrilateral
bool slopecmp(vector<Point>);
//gets the x value between two contours and then returns the amount of pixels that the bot is off
void getCenteredX();
void sendData();

int main() {
	cerr << "Started" << endl;

	globalX = 0;
	xOffDistance = 0;
	xSpread = 0;
	notOld = false;
	Camera.set(CV_CAP_PROP_GAIN, 1);
	Camera.set(CV_CAP_PROP_EXPOSURE, 1);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cerr << "1" << endl;
	Camera.open();
	for (int i = 0; i < 20; i++) {
		cerr << "2" << endl;

		Camera.grab();
		Camera.retrieve(img);
	}
	cout << "3";
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress("roboRIO-1259-FRC.local");
	netTable = NetworkTable::GetTable("OpenCV");
	//int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	//if (fd < 0)
	//{
	//	cerr << "error " << errno << " opening :" << portname << strerror(errno);
	//	return -1;
	//}

	//set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	//set_blocking (fd, 0);                // set no blocking
	netTable->PutNumber("hi", 0);
	if (Camera.isOpened()) {
		//thread first (grabframe);
		cerr << "Opened Camera" << endl;
		//while(img.empty());
		grabframe();
		//thread second(contouringstuffs);
		//thread third (sendData);
		//first.join();
		//second.join();
		//third.join();
	}

	Camera.release();
	return 0;
}

void grabframe() {
	while (true) {
		Camera.grab();
		Camera.retrieve(img);
		contouringstuffs();
		sendData();
	}
}

void contouringstuffs() {
	//while(true) {
	cvtColor(img, hsvimage, CV_BGR2HSV);
	medianBlur(hsvimage, hsvimage, 3);
	inRange(hsvimage, lower, upper, inrange);
//		inRange(img, lower, upper, inrange);

//		cv::imwrite("raspicam1.jpg", img);
		cv::imwrite("raspicam_pic.jpg",inrange);
//		cv::imwrite("raspicam_hsv.bmp", hsvimage);

	findContours(inrange, contours, hierarchy, CV_RETR_LIST,
			CV_CHAIN_APPROX_NONE);
	drawing = Mat::zeros(inrange.size(), CV_8UC3);

	cmpairs.clear();

	for (int i = 0; i < contours.size(); i++) {
		Moments m = moments(contours[i], true);
		int area = m.m00;
		//create a point for the bounding circle's radius and a float for it's center
		//Point2f center;
		//float radius;
		//check to make sure the contour is concave
		if (area > 120) {
			vector<Point> poly;
			approxPolyDP(Mat(contours[i]), poly,
					arcLength(contours[i], true) * 0.05, true);
			if ((poly.size() == 4) /*&& slopecmp(poly)*/) {
				cmpairs.push_back((cmpair_t ) { &contours[i], m });
			}
		}
	}
	//Returns array sorted smallest to biggest
	std::sort(cmpairs.begin(), cmpairs.end(), compareContourAreas);

	//drawContours(drawing, contours, (contours.size() - 1), contourColor1, 2, 8,
	//		hierarchy, 0);
	//drawContours(drawing, contours, (contours.size() - 2), contourColor2, 2, 8,
	//		hierarchy, 0);

//		cv::imwrite("contourImage.jpg", drawing);
	if (cmpairs.size() >= 2) {
		getCenteredX();
//			cerr << xOffDistance;
//			cerr<< "     " ;
//			cerr<<xSpread<<endl;
		foundValidPicture = true;
	} else {
//			cerr <<  "Did not find two contours" << endl;
		Mat rgbimage;
		cvtColor(hsvimage, rgbimage, CV_HSV2BGR);
//			imwrite("failed.jpg",rgbimage);
//			imwrite("failed-thresh.jpg",inrange);
		foundValidPicture = false;
		//cv::imwrite("raspicam_hsv.bmp", hsvimage);

	}
	//}
}

bool slopecmp(vector<Point> poly) {
	double dx0 = ((double) poly[0].x - (double) poly[1].x);
	double dx1 = ((double) poly[1].x - (double) poly[2].x);
	double dx2 = ((double) poly[2].x - (double) poly[3].x);
	double dx3 = ((double) poly[3].x - (double) poly[0].x);
	dx0 = (dx0 == 0) ? 0.00001 : dx0;
	dx1 = (dx1 == 0) ? 0.00001 : dx1;
	dx2 = (dx2 == 0) ? 0.00001 : dx2;
	dx3 = (dx3 == 0) ? 0.00001 : dx3;

	double s0 = ((double) poly[0].y - (double) poly[1].y) / dx0;
	double s1 = ((double) poly[1].y - (double) poly[2].y) / dx1;
	double s2 = ((double) poly[2].y - (double) poly[3].y) / dx2;
	double s3 = ((double) poly[3].y - (double) poly[0].y) / dx3;

	double a0 = 180 - abs(abs(atan(s0) - atan(s2)) - 180);
	double a1 = 180 - abs(abs(atan(s1) - atan(s3)) - 180);

	return a0 < 2.0 && a1 < 2.0;
}

void sendData() {
	if (foundValidPicture) {
		netTable->PutNumber("xPos", xOffDistance);
		netTable->PutNumber("xSpread", xSpread);
		netTable->PutNumber("areaDifference", areaDifference);
		counter++;
		netTable->PutNumber("counter", counter);
	}
}

void getCenteredX() {
	int size = cmpairs.size();
	xSpread = sqrt(
			pow(
					cmpairs[size - 1].m.m10 / cmpairs[size - 1].m.m00
							- cmpairs[size - 2].m.m10 / cmpairs[size - 2].m.m00,2)
					+ pow(
							cmpairs[size - 1].m.m01 / cmpairs[size - 1].m.m00
									- cmpairs[size - 2].m.m01
											/ cmpairs[size - 2].m.m00, 2));
	xOffDistance =
			((cmpairs[size - 1].m.m10 / cmpairs[size - 1].m.m00)
					+ (cmpairs[size - 2].m.m10 / cmpairs[size - 2].m.m00)
					- inrange.cols) / 2;

	areaDifference = ((cmpairs[size-1].m.m10 / cmpairs[size-1].m.m00) < 
			  (cmpairs[size-2].m.m10 / cmpairs[size-2].m.m00)) ? 
			 (cmpairs[size-1].m.m00 - cmpairs[size-2].m.m00) / (cmpairs[size-1].m.m00 + cmpairs[size-2].m.m00):
			(cmpairs[size-2].m.m00 - cmpairs[size-1].m.m00) / (cmpairs[size-1].m.m00 + cmpairs[size-2].m.m00);
}

//Used with sort
bool compareContourAreas(cmpair_t cmpair1, cmpair_t cmpair2) {
	return (cmpair1.m.m00 < cmpair2.m.m00);
}

/*
 int set_interface_attribs (int fd, int speed, int parity) {
 struct termios tty;
 memset (&tty, 0, sizeof tty);
 if (tcgetattr (fd, &tty) != 0)
 {
 cerr << "error "<< errno << " from tcgetattr"<< endl;
 return -1;
 }

 cfsetospeed (&tty, speed);
 cfsetispeed (&tty, speed);

 tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
 // disable IGNBRK for mismatched speed tests; otherwise receive break
 // as \000 chars
 tty.c_iflag &= ~IGNBRK;         // disable break processing
 tty.c_lflag = 0;                // no signaling chars, no echo,
 // no canonical processing
 tty.c_oflag = 0;                // no remapping, no delays
 tty.c_cc[VMIN]  = 0;            // read doesn't block
 tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

 tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

 tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
 // enable reading
 tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
 tty.c_cflag |= parity;
 tty.c_cflag &= ~CSTOPB;
 tty.c_cflag &= ~CRTSCTS;

 if (tcsetattr (fd, TCSANOW, &tty) != 0)
 {
 cerr << "error " << errno << " from tcsetattr"<< endl;
 return -1;
 }
 return 0;
 }

 void set_blocking (int fd, int should_block) {
 struct termios tty;
 memset (&tty, 0, sizeof tty);
 if (tcgetattr (fd, &tty) != 0) {
 cerr << "error " << errno << " from tggetattr"<< endl;
 return;
 }

 tty.c_cc[VMIN]  = should_block ? 1 : 0;
 tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

 if (tcsetattr (fd, TCSANOW, &tty) != 0) {
 cerr << "error " << errno << " setting term attributes" << endl;
 }
 }
 */
