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

using namespace cv;
using namespace std;
using namespace raspicam;

const char* const  portname = "/dev/ttyAMA0";
int32_t globalX;
bool notOld;
int fd;

shared_ptr<NetworkTable> netTable;
Scalar lower = Scalar(40, 100, 100);
Scalar upper = Scalar(55, 255, 255);
Mat frame;
RaspiCam_Cv Camera; //Camera object
Mat streamimage;
Mat hsvimage;
Mat inrange;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
Mat drawing;


Scalar contourColor1 = Scalar(124,252,0);
Scalar contourColor2 = Scalar(180,105,255);
double xOffDistance = 0;
double xSpread = 0;
//int set_interface_attribs(int fd, int speed, int parity);
//void set_blocking(int fd, int should_block);

void grabframe();
//draws 2 largest contours and returns said two largest contours in a vector of contours
void contouringstuffs();
//used with sort in contouringstuffs
bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );
//gets mentioned point from a specific contour
cv::Point getLeftmost(vector<cv::Point> newContour);
cv::Point getRightmost(vector<cv::Point> newContour);
cv::Point getTopmost(vector<cv::Point> newContour);
cv::Point getBotmost(vector<cv::Point> newContour);
//gets the x value between two contours and then returns the amount of pixels that the bot is off
int getCenteredX(vector<vector<Point> > twoContours);
void sendData();

int main() {
	Camera.set(CV_CAP_PROP_GAIN, 1);
	Camera.set(CV_CAP_PROP_EXPOSURE, 1);
	Camera.open();

	for(int i = 0; i < 20; i++) {
		Camera.grab();
	}
	Camera.retrieve(frame);
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

	if(Camera.isOpened()){
		thread first (grabframe);
		cerr <<"Opened Camera"<< endl;
		while (frame.empty());
		thread second (contouringstuffs);
		thread third (sendData);
		first.join();
		second.join();
	} else {
		cerr << "Failed to open camera" << endl;
	}

	Camera.release();
	return 0;
}

void grabframe()
{
	while(true) {
		Camera.grab();
        	Camera.retrieve(frame);
	}
}


void contouringstuffs() {
	while(true) {
		blur(frame,frame,Size(3,3));
		cvtColor(frame, hsvimage, CV_BGR2HSV);
		inRange(hsvimage, lower, upper, inrange);
		cv::imwrite("rapspicam1.jpg", frame);
		cv::imwrite("raspicam_pic.jpg",inrange);

		findContours(inrange, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

		drawing = Mat::zeros( inrange.size(), CV_8UC3 );
		//Returns array sorted smallest to biggest
		std::sort(contours.begin(), contours.end(), compareContourAreas);

     		drawContours( drawing, contours,(contours.size()-1), contourColor1 , 2, 8, hierarchy, 0);
		drawContours( drawing, contours, (contours.size()-2), contourColor2 , 2, 8, hierarchy, 0);

		cv::imwrite("contourImage.jpg", drawing);
		if(!(contours.size() < 2)) {
			if(contourArea(contours.at(contours.size()-2)) > 300) {
				vector<vector<Point> > finalcontours;
				finalcontours.resize(2);
				//do identification of target here... Currently grabs the two largest targets that pass thresholding
				finalcontours.at(0) = contours.at(contours.size()-1);
				finalcontours.at(1) = contours.at(contours.size()-2);
				xOffDistance = (getCenteredX(finalcontours) - (hsvimage.size().width/2)); 
				cerr << xOffDistance << endl;
			        notOld = 1;
			}
			else {
				cerr << "Did not find large enough contours" << endl;
				notOld = 0;
			}
		}
		else {
			cerr <<  "Did not find two contours" << endl;
			notOld = 0;
		}
	}
}

void sendData()
{
	if(notOld)
	{
		netTable->PutNumber("xPos",xOffDistance);
		netTable->PutNumber("xSpread",xSpread);
		netTable ->PutNumber("notOld", 1);
	}
	else
	{
		netTable ->PutNumber("notOld", 0);
	}

}

int getCenteredX(vector<vector<Point> > twoContours) {
	int leftContourRightx = getLeftmost(twoContours.at(0)).x;
	int rightContourLeftx = getLeftmost(twoContours.at(1)).x;
	if(leftContourRightx > rightContourLeftx) {
		std::iter_swap(twoContours.begin(), twoContours.begin()+1);
		rightContourLeftx = leftContourRightx;
		leftContourRightx = getRightmost(twoContours.at(0)).x;
	}
	else {
	leftContourRightx = getRightmost(twoContours.at(0)).x;
	}
	xSpread = (rightContourLeftx - leftContourRightx);
	return (leftContourRightx + xSpread/2);
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
