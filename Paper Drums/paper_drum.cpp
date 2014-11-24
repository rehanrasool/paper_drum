 /*
Author : Omar Waheed, Rehan Rasool
*/

#include "stdafx.h"
#include <iostream>
#include <Windows.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <time.h>
#include <deque>

#include <thread>

using namespace cv;
using namespace std;



void redThread();
void yellowThread();
void greenThread();
void blueThread();
void purpleThread();
/*0 is red, 1 is green, 2 is blue, 3 is purple, 4 is yellow, 5 is white*/
int hHigh[5];
int sHigh[5];
int vHigh[5];

int hLow[5];
int sLow[5];
int vLow[5];


Point green;
vector<Point> outlineGreen;

Point blue;
vector<Point> outlineBlue;

Point yellow;
vector<Point> outlineYellow;
/*new*/
void setHSV(int colorNumber);
void computeObjectAreaAndCenter(vector<Point>& outline, double& area, Point& center);
bool findLargestPurpleObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool show);
bool findLargestRedObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum,bool imshow);
bool findLargestBlueObject(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum,bool imshow);
bool findLargestYellowObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum,bool imshow);
bool findLargestGreenObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum,bool imshow);
bool findLargestWhiteObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
void drawOutline(Mat& image, vector<Point>& outline, int red, int green, int blue);
void setThresh(int colorNumber);
void onTrackbarRed(int value, void* data);
void onTrackbarBlue(int value, void* data);
void onTrackbarYellow(int value, void* data);
void onTrackbarSelect(int colorNum, void* data);
/*new end*/

void convertToGrayScale(Mat& frame); // converts the image to grayscale (1 channel)
void computeDifferenceOfFrames(Mat& currentFrame, Mat& previousFrame, Mat& differenceMatrix); // computes difference of current and previous frames
void onTrackbar(int value, void* data);
void onTrackbar2(int value, void* data);
void smoothImage(Mat& image, double sigma);
Mat diffMatrix;
int thresholdOfDiffMatrix = 30;
int blurQuantity = 1;
/*new*/

int colorNumber = 0; // which color is selected?

int redThreshold = 0;
int largestRedArea = 1;
int currentRedArea = 1;

int currentYellowArea = 1;
int yellowThreshold = 1;
int largestYellowArea = 1;

int currentWhitewArea = 1;
int WhiteThreshold = 1;
int largestWhiteArea = 1;

int blueThreshold = 1;
int largestBlueArea = 1;
int currentBlueArea = 1;

int greenThreshold;
int currentGreenArea = 1;
int largestGreenArea = 1;

int purpleThreshold;
int currentPurpleArea = 1;
int largestPurpleArea = 1;

Mat justRedG;
Mat justGreenG;
Mat justPurplrG;
Mat justYellowG;
Mat justBlueG;


int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;
/*New*/

Point red;
vector<Point> outlineRed;

Point white;
vector<Point> outlineWhite;

Point purple;
vector<Point> outlinePurple;




Mat view, view0, imgThresholded, imgHSV, mask, orientation, previousFrame;

int main(int argc, char* argv[])
{
	VideoCapture capture;
	capture.open(0);
	if (!capture.isOpened())
	{
		int error = -1;
		return 1;
	}

	namedWindow("Camera View", 1);
	namedWindow("Control", CV_WINDOW_FREERATIO); //create a window called "Control" -> controls for HSV values

	// http://docs.opencv.org/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html

	int smoothSlider;
	int smoothSliderMax = 100;

	//Mat view, view0, imgThresholded, imgHSV, mask, orientation, previousFrame;
	float timeStamp = 0;
	Mat motionHistory;
	Mat segMask;
	vector<Rect> boundingRects;
	bool blink = false;
	float MHI_DURATION = 2;
	bool bRecording = false;
	int frameNumber = 0;
	bool horizontalMotion = true;
	bool verticalMotion = true;
	deque<String> historyQueue;
	time_t starttime;
	time_t differenceTime;
	time_t timer;
	/*NEW*/
	
	/*NEW*/

	char directory[128] = { '\0' };
	directory[0] = '.';


	if (argc > 1)
	{
		strcpy_s(directory, argv[1]);
	}
	char filename[256];
	starttime = time(&timer); // time starts here

	//create a slider in the window
	createTrackbar("threshold", "Camera View", &smoothSlider, smoothSliderMax, onTrackbar);
	createTrackbar("Blur", "Camera View", &smoothSlider, smoothSliderMax, onTrackbar2);
	createTrackbar("Red Threshold", "Camera View", &redThreshold, 255, onTrackbarRed);
	createTrackbar("Blue Threshold", "Camera View", &blueThreshold, 255, onTrackbarBlue);
	createTrackbar("yellow Threshold", "Camera View", &yellowThreshold, 255, onTrackbarYellow);
	createTrackbar("Color", "Camera View", &colorNumber, 6, onTrackbarSelect);

	/*Trackers for HSV color detection*/
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
	/*HSV trackers end*/
	// initialze queue with 5 values
	for (int i = 0; i < 4; i++){
		historyQueue.push_front("no output");
	}


	//all of the stuff gets computed in the onTrackbar function so that things get recomputed 
	//when you apply different levels of smoothing
	//Here, we call it manually for initialization
	onTrackbar(30, NULL);
	onTrackbar2(1, NULL);
	onTrackbarRed(0, NULL);
	onTrackbarBlue(0, NULL);

	bool redStarted = false;

	int count = 0;
			
	while (capture.isOpened())
	{
		
		view.copyTo(previousFrame);// saves the previous frame
		capture.read(view0);
		view0.copyTo(view);
		cvtColor(view, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		//motionHistory = cv::Mat::zeros(view.rows, view.cols, CV_32FC1);//
		//mask = cv::Mat::zeros(view.rows, view.cols, CV_8UC1);
		//orientation = cv::Mat::zeros(view.rows, view.cols, CV_32FC1);
		imshow("Camera View", view);
		
		if (bRecording)
		{
			switch (colorNumber){
			
			case 0:
			
					  findLargestRedObject(view, red, outlineRed, redThreshold,0,true);
					  drawOutline(view0, outlineRed, 255, 0, 0);

					  /*thread t1(redThread);
					  t1.join();
					  */
			
				break;
			case 1:
			
					  setHSV(colorNumber);
					  findLargestGreenObject(view, green, outlineGreen, greenThreshold,1,true);
					  drawOutline(view0, outlineGreen, 0, 255, 0);
					  /*
					  thread t3(greenThread);

					  t3.join();
					  */
					  //imshow("green", justGreenG);
			

				break;
			case 2:
			
					  findLargestBlueObject(view, blue, outlineBlue, blueThreshold,2,true);
					  drawOutline(view0, outlineBlue, 0, 0, 255);
					  /*
					  thread t3(blueThread);
					  t3.join();
					  */
			

				break;
			case 3:
			
					  setHSV(colorNumber);
					  findLargestPurpleObject(view, purple, outlinePurple, purpleThreshold,3,true);
					  drawOutline(view0, outlinePurple, 255, 0, 128);


					  /*
					  thread t3(purpleThread);
					  t3.join();
					  */
					//  imshow("purple", justPurplrG);
			
				break;
			case 4:
			
					  setHSV(colorNumber);
					  findLargestYellowObject(view, yellow, outlineYellow, yellowThreshold,4,true);
					  drawOutline(view0, outlineYellow, 255, 255, 0);
					 /* thread t3(yellowThread);
					  imshow("yellow", justYellowG);
					  t3.join();
					  */
					 // imshow("yellow", justYellowG);
			
				break;
			case 5:
				break;
			default:
				if (count == 1){
					destroyWindow("Just Green");
					destroyWindow("Just Yellow");
					destroyWindow("Just Blue");
					destroyWindow("Just Purple");
					destroyWindow("Just Red");
				}
				if (count % 3 == 0){
					findLargestRedObject(view, red, outlineRed, redThreshold, 0, false);
					findLargestPurpleObject(view, purple, outlinePurple, purpleThreshold, 3, false);
					findLargestBlueObject(view, blue, outlineBlue, blueThreshold, 2, false);
					findLargestYellowObject(view, yellow, outlineYellow, yellowThreshold, 4, false);
					findLargestGreenObject(view, green, outlineGreen, greenThreshold, 1, false);
				}
				int percentAreaCoveredRed = 100 - ((currentRedArea * 100) / largestRedArea);
				if (percentAreaCoveredRed >= 15 && percentAreaCoveredRed <= 70){
					PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);
					//cout << "covered area red! = " << percentAreaCoveredRed << endl;
				}
				int percentAreaCoveredBlue = 100 - ((currentBlueArea * 100) / largestBlueArea);
				if (percentAreaCoveredBlue >= 15 && percentAreaCoveredBlue <= 70){
					PlaySound("sound/hihat_hit.wav", NULL, SND_ASYNC);
				}
				count++;
				//drawOutline(view0, outlineGreen,0,255,0);
				//drawOutline(view0, outlineRed, 255,0,0);
			//	drawOutline(view0, outlinePurple, 255, 0, 128);
			//	drawOutline(view0, outlineBlue,0,0,255);
				//drawOutline(view0, outlineYellow, 255,255,0);
				
				
				/*thread t1(redThread);
				thread t2(blueThread);
				thread t3(greenThread);
				thread t4(yellowThread);
				thread t5(purpleThread);
				t1.join();
				t2.join();
				t3.join();
				t4.join();
				t5.join();
				*/
				break;
			}
			//thread t1(redThread);
			//thread t2(blueThread);
			//thread t3(greenThread);
			//thread t4(yellowThread);
			//thread t5(purpleThread);
			//findLargestRedObject(view, red, outlineRed, redThreshold);
			//findLargestPurpleObject(view, purple, outlinePurple, purpleThreshold);
			//findLargestBlueObject(view, blue, outlineBlue, blueThreshold);
			//findLargestYellowObject(view, yellow, outlineYellow, yellowThreshold);
			//findLargestWhiteObject(view, white, outlineWhite, yellowThreshold);
			//findLargestGreenObject(view, green, outlineGreen, greenThreshold);
			//drawOutline(view0, outlineGreen,0,255,0);
			//drawOutline(view0, outlineRed, 255,0,0);
			//drawOutline(view0, outlinePurple, 255, 0, 128);
			//drawOutline(view0, outlineBlue,0,0,255);
			//drawOutline(view0, outlineYellow, 255,255,0);
		//	drawOutline(view0, outlineWhite, 255, 255, 255);
			differenceTime = starttime - time(&timer); // get time difference from start
			timeStamp = (float)abs(differenceTime % 100);
			//inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
			sprintf_s(filename, "%s/frame_%04d.jpg", directory, frameNumber);
			
			/*
			int percentAreaCoveredRed = 100 -( (currentRedArea * 100) / largestRedArea);
			if (percentAreaCoveredRed >= 5 && percentAreaCoveredRed <= 70){
				PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);
				cout << "covered area red! = " << percentAreaCoveredRed << endl;
			}
			*/

			/*
			int percentAreaCoveredBlue = 100 - ((currentBlueArea * 100) / largestBlueArea);
			if (percentAreaCoveredBlue >= 5 && percentAreaCoveredBlue <= 70){
				PlaySound("sound/hihat_hit.wav", NULL, SND_ASYNC);
				cout << "covered area blue! = " << percentAreaCoveredBlue << endl;
			}
			*/

			/*
			int percentAreaCoveredYellow = 100 - ((currentYellowArea * 100) / largestYellowArea);
			if (percentAreaCoveredYellow >= 5 && percentAreaCoveredYellow <= 70){
				PlaySound("sound/tom1_hit.wav", NULL, SND_ASYNC);
				cout << "covered area Yellow! = " << percentAreaCoveredYellow << endl;;
			}
			*/

			/*
			computeDifferenceOfFrames(view, previousFrame, diffMatrix); // computes difference of previous and current frame
			
			threshold(diffMatrix, diffMatrix, thresholdOfDiffMatrix, 40, CV_THRESH_BINARY); // threhold to remove noise
			smoothImage(diffMatrix, blurQuantity);
			updateMotionHistory(diffMatrix, motionHistory, (timeStamp), MHI_DURATION); // update motion history

			calcMotionGradient(motionHistory, mask, orientation, 5, 12500.0, 3); // calculate motion gradient

			segmentMotion(motionHistory, segMask, boundingRects, timeStamp, MHI_DURATION);

			// calculate angle of motion using the history matrix
			double angle;
			angle = calcGlobalOrientation(orientation, mask, motionHistory, timeStamp, MHI_DURATION);

			//cout << "angle = " << angle << "\n";

			// categorize angles into motion categories
			if ((angle <= 100 && angle >= 80) || (angle <= 280 && angle >= 260)){
			historyQueue.pop_back();
			historyQueue.push_front("Horizontal motion");
			}
			else if ((angle <= 10 && angle >= 3) || (angle <= 360 && angle >= 350)){
			historyQueue.pop_back();
			historyQueue.push_front("Vertical motion");
			}

			// only accept a certain motion if 5 consecutive motions are the same (this is to avoid small categorizations from natural shaking etc.)
			for (int i = 0; i < historyQueue.size(); i++){
			if (historyQueue[i] != "Horizontal motion"){
			horizontalMotion = false;
			}
			if (historyQueue[i] != "Vertical motion"){
			verticalMotion = false;
			}
			}

			// print the appropriate case when true
			if (horizontalMotion){
			cout << "Horizontal motion" << endl;
			}
			else if (verticalMotion){
			cout << "Vertical motion" << endl;
			}

			// reset values to default for next frame
			horizontalMotion = true;
			verticalMotion = true;

			for (int i = 0; i< boundingRects.size(); i++) // this will put rectangles around objects that moved.
			{
			Scalar color = Scalar(255, 0, 255);
			rectangle(view0, boundingRects[i].tl(), boundingRects[i].br(), color, 3, 8, 0);
			}

			imshow("MHI pic", motionHistory);
			imshow("difference pic", diffMatrix);
			imshow("view0", view0);
			imwrite(filename, view0); // uncomment this to save a series of frames of detected object
			MOTION*/
			
			imshow("view0", view0);
			//t1.join();
			//t2.join();
			//t3.join();
			//t4.join();
			//t5.join();
			
			//imshow("GREEEEN", justGreen[0]);
			//imshow("Thresholded Image", imgThresholded); //show the thresholded image
			frameNumber++;
		}

		char key = waitKey(33);
		if (key == 'p')
		{

			sprintf_s(filename, "%s/frame_%04d.jpg", directory, frameNumber);
			convertToGrayScale(view);
			computeDifferenceOfFrames(view, previousFrame, diffMatrix);

			imwrite(filename, diffMatrix);
			frameNumber++;
		}
		if (key == ' ')
		{
			bRecording = !bRecording;
		}
		if (key == 's'){ // when threshold is good then press s i.e done
			setThresh(colorNumber);
		}
		if (key == 'q')
		{
			break;
		}
	}

	return 0;
}

void setHSV(int colorNumber){
	switch (colorNumber)
	{
	case 0:
		hHigh[colorNumber] = iHighH;
		sHigh[colorNumber] = iHighS;
		vHigh[colorNumber] = iHighV;
		hLow[colorNumber] = iLowH;
		sLow[colorNumber] = iLowS;
		vLow[colorNumber] = iLowV;
		break;
	case 1:
		hHigh[colorNumber] = iHighH;
		sHigh[colorNumber] = iHighS;
		vHigh[colorNumber] = iHighV;
		hLow[colorNumber] = iLowH;
		sLow[colorNumber] = iLowS;
		vLow[colorNumber] = iLowV;
		break;

	case 2:
		hHigh[colorNumber] = iHighH;
		sHigh[colorNumber] = iHighS;
		vHigh[colorNumber] = iHighV;
		hLow[colorNumber] = iLowH;
		sLow[colorNumber] = iLowS;
		vLow[colorNumber] = iLowV;
		break;

	case 3:
		hHigh[colorNumber] = iHighH;
		sHigh[colorNumber] = iHighS;
		vHigh[colorNumber] = iHighV;
		hLow[colorNumber] = iLowH;
		sLow[colorNumber] = iLowS;
		vLow[colorNumber] = iLowV;
		break;
	case 4:
		hHigh[colorNumber] = iHighH;
		sHigh[colorNumber] = iHighS;
		vHigh[colorNumber] = iHighV;
		hLow[colorNumber] = iLowH;
		sLow[colorNumber] = iLowS;
		vLow[colorNumber] = iLowV;
		break;

	case 5:
		hHigh[colorNumber] = iHighH;
		sHigh[colorNumber] = iHighS;
		vHigh[colorNumber] = iHighV;
		hLow[colorNumber] = iLowH;
		sLow[colorNumber] = iLowS;
		vLow[colorNumber] = iLowV;
		break;

	default:
		break;
	}
}

void setThresh(int colorNumber){
	switch (colorNumber){
	
	case 0 :
		largestRedArea = currentRedArea;
		cout << "red set";
		break;
	case 1:
		largestGreenArea = currentGreenArea;
		break;
	case 2: 
		largestBlueArea = currentBlueArea;
		break;
	case 3:
		largestPurpleArea = currentPurpleArea;
		break;
	case 4:
		largestYellowArea = currentYellowArea;
	case 5 :
		//largestWhiteArea = currentWhiteArea;
		break;
	default:
		break;

	}
	
}


/*Threading*/

void purpleThread() {
	//while (true){
	if (!view.empty() && !view0.empty()){
		if (view.size > 0 && view0.size > 0){
			findLargestPurpleObject(view, red, outlinePurple, redThreshold,3,false);
			drawOutline(view0, outlinePurple, 255, 0, 0);
			Sleep(10);
			int percentAreaCoveredPurple = 100 - ((currentPurpleArea * 100) / largestPurpleArea);
			if (percentAreaCoveredPurple >= 15 && percentAreaCoveredPurple <= 70){
				PlaySound("sound/floortom_hit.wav", NULL, SND_ASYNC);
				cout << "covered area red! = " << percentAreaCoveredPurple << endl;
			}
			//imshow("view0", view0);
		}
	}
	//}

}

void redThread() {
	//while (true){
		if (!view.empty() && !view0.empty()){
			if (view.size > 0 && view0.size > 0){
				findLargestRedObject(view, red, outlineRed, redThreshold, 0,false);
				drawOutline(view0, outlineRed, 255, 0, 0);
				int percentAreaCoveredRed = 100 - ((currentRedArea * 100) / largestRedArea);
				if (percentAreaCoveredRed >= 15 && percentAreaCoveredRed <= 70){
					PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);
					cout << "covered area red! = " << percentAreaCoveredRed << endl;
				}
				//imshow("view0", view0);
			}
		}
	//}

}

void blueThread() {
	//while (true){
		if (!view.empty() && !view0.empty()){
			if (view.size > 0 && view0.size > 0){
				findLargestBlueObject(view, blue, outlineBlue, blueThreshold, 2,false);
				drawOutline(view0, outlineBlue, 0, 0, 255);
				Sleep(10);
				int percentAreaCoveredBlue = 100 - ((currentBlueArea * 100) / largestBlueArea);
				if (percentAreaCoveredBlue >= 15 && percentAreaCoveredBlue <= 70){
					PlaySound("sound/hihat_hit.wav", NULL, SND_ASYNC);
					cout << "covered area blue! = " << percentAreaCoveredBlue << endl;
				}
				//imshow("view0", view0);
			}
		}
	//}

}

void greenThread() {
	//while (true){
		if (!view.empty() && !view0.empty()){
			if (view.size > 0 && view0.size > 0){
				findLargestGreenObject(view, green, outlineGreen, greenThreshold,1,false);
				drawOutline(view0, outlineGreen, 0, 255, 0);
				Sleep(10);
				//imshow("view0", view0);
			}
		}
		
	//}

}

void yellowThread() {
	//while (true){
		if (!view.empty() && !view0.empty()){
			if (view.size > 0 && view0.size > 0){
				findLargestYellowObject(view, yellow, outlineYellow, yellowThreshold,4,false);
				drawOutline(view0, outlineYellow, 255, 255, 0);
				Sleep(10);
				int percentAreaCoveredYellow = 100 - ((currentYellowArea * 100) / largestYellowArea);
				if (percentAreaCoveredYellow >= 15 && percentAreaCoveredYellow <= 70){
					PlaySound("sound/tom1_hit.wav", NULL, SND_ASYNC);
					cout << "covered area Yellow! = " << percentAreaCoveredYellow << endl;;
				}
				//imshow("view0", view0);
			}
		}
		//}
	}


/**/


// convert a frame to grayscale
void convertToGrayScale(Mat& frame){
	Mat tempGray;
	cvtColor(frame, tempGray, CV_BGR2GRAY);
	tempGray.copyTo(frame);
}

// compute difference between consecutive frames to detect motion
void computeDifferenceOfFrames(Mat& currentFrame, Mat& previousFrame, Mat& differenceMatrix){
	Mat prevFrameTemp;
	Mat tempcurrentFrame;
	currentFrame.copyTo(tempcurrentFrame);
	convertToGrayScale(tempcurrentFrame);
	previousFrame.copyTo(prevFrameTemp);
	convertToGrayScale(prevFrameTemp);



	currentFrame.copyTo(differenceMatrix);
	absdiff(tempcurrentFrame, prevFrameTemp, differenceMatrix);




}

void smoothImage(Mat& image, double sigma)
{
	if (sigma <= 0.0)
	{
		return;
	}
	//smooth the image 
	// This is another example of a convolution / filtering operation, this time with a 
	// Gaussian kernel. You could also use all ones to get the mean of the pixels in the image
	GaussianBlur(image, image, Size(0, 0), sigma, sigma, BORDER_DEFAULT);
}

void onTrackbar(int value, void* data)
{

	thresholdOfDiffMatrix = value;

}

void onTrackbar2(int value, void* data)
{

	blurQuantity = value;

}

void onTrackbarRed(int value, void* data)
{

	redThreshold = value;

}


void onTrackbarBlue(int value, void* data)
{

	blueThreshold = value;

}

void onTrackbarYellow(int value, void* data)
{

	yellowThreshold = value;

}

void onTrackbarSelect(int colorNum, void* data)
{
	colorNumber = colorNumber;
}

void drawOutline(Mat& image, vector<Point>& outline,int blue,int green,int red)
{
	int numPoints = outline.size() - 1;
	for (int f = 0; f<numPoints; f++)
	{
		line(image, outline[f], outline[f + 1], Scalar(red, green, blue), 3);
	}
}

bool findLargestRedObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
{
	//allocate some images to store intermediate results
	vector<Mat> YCrCb;
	YCrCb.push_back(Mat(view.rows, view.cols, CV_8UC3));
	vector<Mat> justRed;
	justRed.push_back(Mat(view.rows, view.cols, CV_8UC1));
	vector<Mat> displayRed;
	displayRed.push_back(Mat(view.rows, view.cols, CV_8UC3));

	//Switch color spaces to YCrCb so we can detect red objects even if they are dark
	cvtColor(view, YCrCb[0], CV_BGR2YCrCb);

	//Pull out just the red channel
	int extractRed[6] = { 1, 0, 1, 1, 1, 2 };
	mixChannels(&(YCrCb[0]), 1, &(justRed[0]), 1, extractRed, 1);

	// Threshold the red object (with the threshold from the slider)
	threshold(justRed[0], justRed[0], redThreshold, 255, CV_THRESH_BINARY);
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;

	//Find all of the contiguous image regions
	findContours(justRed[0], objectContours, dummy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	if (imshow){
	cv::imshow("Just Red", justRed[0]);
	}
	//find the largest object
	int largestArea(-1), largestIndex(-1);
	Point largestCenter;
	for (int i = 0; i<objectContours.size(); i++)
	{
		Point tempCenter;
		double tempArea;
		computeObjectAreaAndCenter(objectContours[i], tempArea, tempCenter);

		if (tempArea > largestArea)
		{
			largestArea = tempArea;
			largestIndex = i;
			largestCenter = tempCenter;
			currentRedArea = largestArea;
		}
	}
	location = largestCenter;
	if (largestIndex >= 0)
	{
		outline = objectContours[largestIndex];
	}

	//Construct an image for display that shows the red channel as gray
	mixChannels(&(YCrCb[0]), 1, &(displayRed[0]), 1, extractRed, 3);
	if (largestIndex >= 0)
	{
		//put a red circle around the red object
		circle(displayRed[0], largestCenter, std::min(double(view.cols) / 2, sqrt(largestArea)), Scalar(0, 0, 255), 1);
	}
	//imshow("Just Red", displayRed[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

void computeObjectAreaAndCenter(vector<Point>& outline, double& area, Point& center)
{
	// http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
	Moments objectProperties;
	objectProperties = moments(outline, false);

	area = objectProperties.m00;
	center.x = (objectProperties.m10 / area);
	center.y = (objectProperties.m01 / area);
}

bool findLargestBlueObject(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow)
{
	//allocate some images to store intermediate results
	vector<Mat> YCrCb;
	YCrCb.push_back(Mat(view.rows, view.cols, CV_8UC3));
	vector<Mat> justBlue;
	justBlue.push_back(Mat(view.rows, view.cols, CV_8UC1));
	vector<Mat> displayBlue;
	displayBlue.push_back(Mat(view.rows, view.cols, CV_8UC3));

	//Switch color spaces to YCrCb so we can detect red objects even if they are dark
	cvtColor(view, YCrCb[0], CV_BGR2YCrCb);

	//Pull out just the red channel
	int extractBlue[6] = { 2, 0, 2, 1, 2, 2 };
	mixChannels(&(YCrCb[0]), 1, &(justBlue[0]), 1, extractBlue, 1);

	// Threshold the red object (with the threshold from the slider)
	threshold(justBlue[0], justBlue[0], blueThreshold, 255, CV_THRESH_BINARY );
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	if(imshow){
		cv::imshow("Just Blue", justBlue[0]);
	}
	//Find all of the contiguous image regions
	findContours(justBlue[0], objectContours, dummy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//find the largest object
	int largestArea(-1), largestIndex(-1);
	Point largestCenter;
	for (int i = 0; i<objectContours.size(); i++)
	{
		Point tempCenter;
		double tempArea;
		computeObjectAreaAndCenter(objectContours[i], tempArea, tempCenter);

		if (tempArea > largestArea)
		{
			largestArea = tempArea;
			largestIndex = i;
			largestCenter = tempCenter;
			currentBlueArea = largestArea;
		}
	}
	location = largestCenter;
	if (largestIndex >= 0)
	{
		outline = objectContours[largestIndex];
	}

	//Construct an image for display that shows the red channel as gray
	mixChannels(&(YCrCb[0]), 1, &(displayBlue[0]), 1, extractBlue, 3);
	if (largestIndex >= 0)
	{
		//put a red circle around the red object
		circle(displayBlue[0], largestCenter, std::min(double(view.cols) / 2, sqrt(largestArea)), Scalar(0, 0, 255), 1);
	}
	//imshow("Just blue", displayBlue[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool findLargestYellowObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum,bool imshow)
{
	//allocate some images to store intermediate results
	vector<Mat> YCrCb;
	YCrCb.push_back(Mat(view.rows, view.cols, CV_8UC3));
	vector<Mat> justYellow;
	justYellow.push_back(Mat(view.rows, view.cols, CV_8UC1));
	vector<Mat> displayYellow;
	displayYellow.push_back(Mat(view.rows, view.cols, CV_8UC3));

	//Switch color spaces to YCrCb so we can detect red objects even if they are dark
	cvtColor(view, YCrCb[0], CV_BGR2YCrCb);

	//Pull out just the red channel
	//int extractRed[6] = { 1, 0, 1, 1, 1, 2 };
	//mixChannels(&(YCrCb[0]), 1, &(justRed[0]), 1, extractRed, 1);

	// Threshold the red object (with the threshold from the slider)

	
	//inRange(YCrCb[0], Scalar(22, 113, 112), Scalar(32, 209, 255), justYellow[0]); //Threshold the image
	//inRange(YCrCb[0], Scalar(16, 94, 112), Scalar(47, 174, 223), justYellow[0]);
	inRange(YCrCb[0], Scalar(hLow[colorNum], sLow[colorNum], vLow[colorNum]), Scalar(hHigh[colorNum], sHigh[colorNum], vHigh[colorNum]), justYellow[0]);

	//imshow("yellow before", justYellow[0]);
	equalizeHist(justYellow[0], justYellow[0]);
	//imshow("yellow after", justYellow[0]);
	threshold(justYellow[0], justYellow[0],yellowThreshold,255, CV_THRESH_BINARY);
	if (imshow){
		cv::imshow("Just Yellow", justYellow[0]);
	}
	//justYellowG = justYellow[0];

	//************************
	vector<Mat> channels;
	split(YCrCb[0], channels);

	equalizeHist(channels[0], channels[0]);

	Mat result;
	merge(channels, YCrCb[0]);

	cvtColor(YCrCb[0], result, CV_YCrCb2BGR);


	//************************8
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	//imshow("HSV REDDDD", justYellow[0]);

	//Find all of the contiguous image regions
	findContours(justYellow[0], objectContours, dummy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//find the largest object
	int largestArea(-1), largestIndex(-1);
	Point largestCenter;
	for (int i = 0; i<objectContours.size(); i++)
	{
		Point tempCenter;
		double tempArea;
		computeObjectAreaAndCenter(objectContours[i], tempArea, tempCenter);

		if (tempArea > largestArea)
		{
			largestArea = tempArea;
			largestIndex = i;
			largestCenter = tempCenter;
			currentYellowArea = largestArea;
		}
	}
	location = largestCenter;
	if (largestIndex >= 0)
	{
		outline = objectContours[largestIndex];
	}

	//Construct an image for display that shows the red channel as gray
	//mixChannels(&(YCrCb[0]), 1, &(displayRed[0]), 1, extractRed, 3);
	if (largestIndex >= 0)
	{
		//put a red circle around the red object
		circle(displayYellow[0], largestCenter, std::min(double(view.cols) / 2, sqrt(largestArea)), Scalar(0, 0, 255), 1);
	}
	//imshow("Just yellow", displayYellow[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool findLargestGreenObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
{
	//allocate some images to store intermediate results
	vector<Mat> YCrCb;
	YCrCb.push_back(Mat(view.rows, view.cols, CV_8UC3));
	Vector<Mat> justGreen;
	justGreen.push_back(Mat(view.rows, view.cols, CV_8UC1));
	vector<Mat> displayGreen;
	displayGreen.push_back(Mat(view.rows, view.cols, CV_8UC3));

	//Switch color spaces to YCrCb so we can detect red objects even if they are dark
	cvtColor(view, YCrCb[0], CV_BGR2HSV);

	//Pull out just the red channel
	//int extractRed[6] = { 1, 0, 1, 1, 1, 2 };
	//mixChannels(&(YCrCb[0]), 1, &(justRed[0]), 1, extractRed, 1);

	// Threshold the red object (with the threshold from the slider)

	//inRange(YCrCb[0], Scalar(39, 4, 28), Scalar(98, 161, 251), justGreen[0]); //Threshold the image
	inRange(YCrCb[0], Scalar(hLow[colorNum], sLow[colorNum], vLow[colorNum]), Scalar(hHigh[colorNum], sHigh[colorNum], vHigh[colorNum]), justGreen[0]); //Threshold the image
	//inRange(view, Scalar(47, 82, 0), Scalar(111, 255, 103), justGreen[0]); //Threshold the image lib
	//imshow("green before", justGreen[0]);
	equalizeHist(justGreen[0], justGreen[0]);
	//imshow("green after", justGreen[0]);
	threshold(justGreen[0], justGreen[0], 0, 255, THRESH_BINARY);
	//imshow("GREEN", justGreen[0]);
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	if (imshow){
		cv::imshow("Just Green", justGreen[0]);
	}
	//justGreenG = justGreen[0];
	//Find all of the contiguous image regions
	findContours(justGreen[0], objectContours, dummy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//find the largest object
	int largestArea(-1), largestIndex(-1);
	Point largestCenter;
	for (int i = 0; i<objectContours.size(); i++)
	{
		Point tempCenter;
		double tempArea;
		computeObjectAreaAndCenter(objectContours[i], tempArea, tempCenter);

		if (tempArea > largestArea)
		{
			largestArea = tempArea;
			largestIndex = i;
			largestCenter = tempCenter;
			currentGreenArea = largestArea;
		}
	}
	location = largestCenter;
	if (largestIndex >= 0)
	{
		outline = objectContours[largestIndex];
	}

	//Construct an image for display that shows the red channel as gray
	//mixChannels(&(YCrCb[0]), 1, &(displayRed[0]), 1, extractRed, 3);
	if (largestIndex >= 0)
	{
		//put a red circle around the red object
		circle(displayGreen[0], largestCenter, std::min(double(view.cols) / 2, sqrt(largestArea)), Scalar(0, 0, 255), 1);
	}
	//imshow("Just Green", displayGreen[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool findLargestPurpleObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
{
	//allocate some images to store intermediate results
	vector<Mat> YCrCb;
	YCrCb.push_back(Mat(view.rows, view.cols, CV_8UC3));
	vector<Mat> justPurple;
	vector<Mat> displayPurple;
	justPurple.push_back(Mat(view.rows, view.cols, CV_8UC1));
	displayPurple.push_back(Mat(view.rows, view.cols, CV_8UC3));

	//Switch color spaces to YCrCb so we can detect red objects even if they are dark
	cvtColor(view, YCrCb[0], CV_BGR2HSV);

	//Pull out just the red channel
	//int extractRed[6] = { 1, 0, 1, 1, 1, 2 };
	//mixChannels(&(YCrCb[0]), 1, &(justRed[0]), 1, extractRed, 1);

	// Threshold the red object (with the threshold from the slider)

	//inRange(YCrCb[0], Scalar(39, 4, 28), Scalar(98, 161, 251), justGreen[0]); //Threshold the image
	inRange(YCrCb[0], Scalar(hLow[colorNum], sLow[colorNum], vLow[colorNum]), Scalar(hHigh[colorNum], sHigh[colorNum], vHigh[colorNum]), justPurple[0]); //Threshold the image
	//inRange(YCrCb[0], Scalar(138, 83, 147), Scalar(179, 170, 147), justPurple[0]); //Threshold the image
	//inRange(view, Scalar(47, 82, 0), Scalar(111, 255, 103), justGreen[0]); //Threshold the image lib
	threshold(justPurple[0], justPurple[0], redThreshold, 255, CV_THRESH_BINARY);
	if (imshow)
	cv::imshow("Just Purple", justPurple[0]);
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	//imshow("HSV Greeeen", justGreen[0]);
	justPurplrG = justPurple[0];
	//Find all of the contiguous image regions
	findContours(justPurple[0], objectContours, dummy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//find the largest object
	int largestArea(-1), largestIndex(-1);
	Point largestCenter;
	for (int i = 0; i<objectContours.size(); i++)
	{
		Point tempCenter;
		double tempArea;
		computeObjectAreaAndCenter(objectContours[i], tempArea, tempCenter);

		if (tempArea > largestArea)
		{
			largestArea = tempArea;
			largestIndex = i;
			largestCenter = tempCenter;
			currentPurpleArea = largestArea;
		}
	}
	location = largestCenter;
	if (largestIndex >= 0)
	{
		outline = objectContours[largestIndex];
	}

	//Construct an image for display that shows the red channel as gray
	//mixChannels(&(YCrCb[0]), 1, &(displayRed[0]), 1, extractRed, 3);
	if (largestIndex >= 0)
	{
		//put a red circle around the red object
		circle(displayPurple[0], largestCenter, std::min(double(view.cols) / 2, sqrt(largestArea)), Scalar(0, 0, 255), 1);
	}
	//imshow("Just purple", displayPurple[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool findLargestWhiteObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
{
	//allocate some images to store intermediate results
	vector<Mat> YCrCb;
	YCrCb.push_back(Mat(view.rows, view.cols, CV_8UC3));
	vector<Mat> justWhite;
	justWhite.push_back(Mat(view.rows, view.cols, CV_8UC1));
	vector<Mat> displayWhite;
	displayWhite.push_back(Mat(view.rows, view.cols, CV_8UC3));

	//Switch color spaces to YCrCb so we can detect red objects even if they are dark
	cvtColor(view, YCrCb[0], CV_BGR2YCrCb);

	//Pull out just the red channel
	//int extractRed[6] = { 1, 0, 1, 1, 1, 2 };
	//mixChannels(&(YCrCb[0]), 1, &(justRed[0]), 1, extractRed, 1);

	// Threshold the red object (with the threshold from the slider)


	//inRange(YCrCb[0], Scalar(22, 113, 112), Scalar(32, 209, 255), justYellow[0]); //Threshold the image
	//inRange(YCrCb[0], Scalar(16, 94, 112), Scalar(47, 174, 223), justYellow[0]);
	inRange(YCrCb[0], Scalar(hLow[colorNum], sLow[colorNum], vLow[colorNum]), Scalar(hHigh[colorNum], sHigh[colorNum], vHigh[colorNum]), justWhite[0]);

	//imshow("White before", justWhite[0]);
	//equalizeHist(justWhite[0], justWhite[0]);
	//imshow("White after", justWhite[0]);
	threshold(justWhite[0], justWhite[0], yellowThreshold, 255, CV_ADAPTIVE_THRESH_MEAN_C || CV_THRESH_OTSU);
	
	//imshow("White!!!", justWhite[0]);


	//************************
	/*vector<Mat> channels;
	split(YCrCb[0], channels);

	equalizeHist(channels[0], channels[0]);

	Mat result;
	merge(channels, YCrCb[0]);

	cvtColor(YCrCb[0], result, CV_YCrCb2BGR);
	imshow("YCrCB to RGB", result);
	imshow("YCrCB to RGB YCR", YCrCb[0]);
	*/


	//************************8
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	//imshow("HSV REDDDD", justYellow[0]);

	//Find all of the contiguous image regions
	findContours(justWhite[0], objectContours, dummy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//find the largest object
	int largestArea(-1), largestIndex(-1);
	Point largestCenter;
	for (int i = 0; i<objectContours.size(); i++)
	{
		Point tempCenter;
		double tempArea;
		computeObjectAreaAndCenter(objectContours[i], tempArea, tempCenter);

		if (tempArea > largestArea)
		{

			largestArea = tempArea;
			largestIndex = i;
			largestCenter = tempCenter;
			currentYellowArea = largestArea;
		}
	}
	location = largestCenter;
	if (largestIndex >= 0)
	{
		outline = objectContours[largestIndex];
	}

	//Construct an image for display that shows the red channel as gray
	//mixChannels(&(YCrCb[0]), 1, &(displayRed[0]), 1, extractRed, 3);
	if (largestIndex >= 0)
	{
		//put a red circle around the red object
		circle(displayWhite[0], largestCenter, std::min(double(view.cols) / 2, sqrt(largestArea)), Scalar(0, 0, 255), 1);
	}
	//cv::imshow("Just Yellow", displayWhite[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}