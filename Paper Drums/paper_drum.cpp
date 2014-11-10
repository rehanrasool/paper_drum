/**
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
#include <deque>;

using namespace cv;
using namespace std;


/*new*/

void computeObjectAreaAndCenter(vector<Point>& outline, double& area, Point& center);
bool findLargestRedObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold);
bool findLargestBlueObject(Mat& view, Point& location, vector<Point>& outline, int blueThreshold);
bool findLargestYellowObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold);
void drawOutline(Mat& image, vector<Point>& outline);
void onTrackbarRed(int value, void* data);
void onTrackbarBlue(int value, void* data);
void onTrackbarYellow(int value, void* data);
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
int redThreshold = 0;
int largestRedArea = 1;
int currentRedArea = 1;

int currentYellowArea = 1;
int yellowThreshold = 1;
int largestYellowArea = 1;

int blueThreshold = 1;
int largestBlueArea = 1;
int currentBlueArea = 1;


int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;
/*New*/


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
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control" -> controls for HSV values

	// http://docs.opencv.org/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html

	int smoothSlider;
	int smoothSliderMax = 100;

	Mat view, view0, imgThresholded, imgHSV, mask, orientation, previousFrame;
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
	Point red;
	vector<Point> outlineRed;

	Point blue;
	vector<Point> outlineBlue;

	Point yellow;
	vector<Point> outlineYellow;
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

	while (capture.isOpened())
	{
		
		view.copyTo(previousFrame);// saves the previous frame
		capture.read(view0);
		view0.copyTo(view);
		cvtColor(view, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		motionHistory = cv::Mat::zeros(view.rows, view.cols, CV_32FC1);//
		mask = cv::Mat::zeros(view.rows, view.cols, CV_8UC1);
		orientation = cv::Mat::zeros(view.rows, view.cols, CV_32FC1);
		imshow("Camera View", view);

		if (bRecording)
		{

			findLargestRedObject(view, red, outlineRed, redThreshold);
			findLargestBlueObject(view, blue, outlineBlue, blueThreshold);
			findLargestYellowObject(view, yellow, outlineYellow, yellowThreshold);
			drawOutline(view0, outlineRed);
			drawOutline(view0, outlineBlue);
			drawOutline(view0, outlineYellow);
			differenceTime = starttime - time(&timer); // get time difference from start
			timeStamp = (float)abs(differenceTime % 100);
			inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
			sprintf_s(filename, "%s/frame_%04d.jpg", directory, frameNumber);
			int percentAreaCoveredRed = (currentRedArea * 100) / largestRedArea;

			if (percentAreaCoveredRed >= 50 && percentAreaCoveredRed <= 91){
				PlaySound("sound/snare_hit.wav", NULL, SND_FILENAME);
				cout << "HIT!";
			}

			int percentAreaCoveredBlue = (currentBlueArea * 100) / largestBlueArea;
			if (percentAreaCoveredBlue >= 50 && percentAreaCoveredBlue <= 93){
				PlaySound("sound/hihat_hit.wav", NULL, SND_FILENAME);
				cout << "HIT!";
			}

			int percentAreaCoveredYellow = (currentYellowArea * 100) / largestYellowArea;
			if (percentAreaCoveredYellow >= 50 && percentAreaCoveredYellow <= 93){
				PlaySound("sound/tom1_hit.wav", NULL, SND_FILENAME);
				cout << "HIT!";
			}

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
			imshow("Thresholded Image", imgThresholded); //show the thresholded image
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
		if (key == 'd'){ // when threshold is good then press d i.e done
			largestRedArea = currentRedArea;
			largestBlueArea = currentBlueArea;
			largestYellowArea = currentYellowArea;
		}
		if (key == 'q')
		{
			break;
		}
	}

	return 0;
}

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

void drawOutline(Mat& image, vector<Point>& outline)
{
	int numPoints = outline.size() - 1;
	for (int f = 0; f<numPoints; f++)
	{
		line(image, outline[f], outline[f + 1], Scalar(255, 0, 0), 3);
	}
}

bool findLargestRedObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold)
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
	imshow("Just Red", displayRed[0]);


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

bool findLargestBlueObject(Mat& view, Point& location, vector<Point>& outline, int blueThreshold)
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
	threshold(justBlue[0], justBlue[0], blueThreshold, 255, CV_THRESH_BINARY);
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;

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
	imshow("Just blue", displayBlue[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool findLargestYellowObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold)
{
	//allocate some images to store intermediate results
	vector<Mat> YCrCb;
	YCrCb.push_back(Mat(view.rows, view.cols, CV_8UC3));
	vector<Mat> justYellow;
	justYellow.push_back(Mat(view.rows, view.cols, CV_8UC1));
	vector<Mat> displayYellow;
	displayYellow.push_back(Mat(view.rows, view.cols, CV_8UC3));

	//Switch color spaces to YCrCb so we can detect red objects even if they are dark
	cvtColor(view, YCrCb[0], CV_BGR2HSV);

	//Pull out just the red channel
	//int extractRed[6] = { 1, 0, 1, 1, 1, 2 };
	//mixChannels(&(YCrCb[0]), 1, &(justRed[0]), 1, extractRed, 1);

	// Threshold the red object (with the threshold from the slider)

	//threshold(justRed[0], justRed[0], redThreshold, 255, CV_THRESH_BINARY);
	inRange(view, Scalar(29, 141, 0), Scalar(134, 255, 255), justYellow[0]); //Threshold the image
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
	imshow("Just yellow", displayYellow[0]);


	if (largestIndex >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}