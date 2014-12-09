 /*
Author : Omar Waheed, Rehan Rasool
*/

// sound library


#include "stdafx.h"
#include <iostream>
#include <Windows.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <time.h>
#include <deque>




#include "include/irrKlang.h"

using namespace std;
using namespace irrklang;

#pragma comment(lib, "irrKlang.lib")



#include <thread>

using namespace cv;
using namespace std;

/*0 is red, 1 is green, 2 is blue, 3 is purple, 4 is yellow, 5 is white*/
int hHigh[5];
int sHigh[5];
int vHigh[5];

int hLow[5];
int sLow[5];
int vLow[5];

int hHigh2[5];
int sHigh2[5];
int vHigh2[5];

int hLow2[5];
int sLow2[5];
int vLow2[5];

// offsets
int redPlayedOffset = 0;
int bluePlayedOffset = 0;
int greenPlayedOffset = 0;

Point green;
vector<Point> outlineGreen;

Point blue;
vector<Point> outlineBlue;

Point yellow;
vector<Point> outlineYellow;

Point green2;
vector<Point> outlineGreen2;

Point blue2;
vector<Point> outlineBlue2;

Point yellow2;
vector<Point> outlineYellow2;

/*new*/
cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows);
void setHSV(int colorNumber);
void computeObjectAreaAndCenter(vector<Point>& outline, double& area, Point& center);
bool findLargestPurpleObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool show);
bool findLargestRedObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestRedObject2(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestBlueObject(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum,bool imshow);
bool findLargestBlueObject2(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow);
bool findLargestYellowObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum,bool imshow);
bool findLargestGreenObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum,bool imshow);
bool findLargestGreenObject2(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestWhiteObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
void drawOutline(Mat& image, vector<Point>& outline, int red, int green, int blue);
void setThresh(int colorNumber);
void onTrackbarRed(int value, void* data);
void onTrackbarRed2(int value, void* data);
void onTrackbarBlue(int value, void* data);
void onTrackbarBlue2(int value, void* data);
void onTrackbarYellow(int value, void* data);
void onTrackbarSelect(int colorNum, void* data);
/*new end*/

void onTrackbar(int value, void* data);
void onTrackbar2(int value, void* data);
Mat diffMatrix;
int thresholdOfDiffMatrix = 30;
int blurQuantity = 1;
/*new*/

int colorNumber = 0; // which color is selected?

int redThreshold = 0;
int largestRedArea = 1;
int currentRedArea = 1;
int redThreshold2 = 0;
int largestRedArea2 = 1;
int currentRedArea2 = 1;

int currentYellowArea = 1;
int yellowThreshold = 1;
int largestYellowArea = 1;
int currentYellowArea2 = 1;
int yellowThreshold2 = 1;
int largestYellowArea2 = 1;

int currentWhitewArea = 1;
int WhiteThreshold = 1;
int largestWhiteArea = 1;
int currentWhitewArea2 = 1;
int WhiteThreshold2 = 1;
int largestWhiteArea2 = 1;

int blueThreshold = 1;
int largestBlueArea = 1;
int currentBlueArea = 1;
int blueThreshold2 = 1;
int largestBlueArea2 = 1;
int currentBlueArea2 = 1;

int greenThreshold;
int currentGreenArea = 1;
int largestGreenArea = 1;
int greenThreshold2;
int currentGreenArea2 = 1;
int largestGreenArea2 = 1;

int purpleThreshold;
int currentPurpleArea = 1;
int largestPurpleArea = 1;
int purpleThreshold2;
int currentPurpleArea2 = 1;
int largestPurpleArea2 = 1;

Mat justRedG;
Mat justGreenG;
Mat justPurplrG;
Mat justYellowG;
Mat justBlueG;

Mat justRedG2;
Mat justGreenG2;
Mat justPurplrG2;
Mat justYellowG2;
Mat justBlueG2;


int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

int iLowH2 = 0;
int iHighH2 = 179;

int iLowS2 = 0;
int iHighS2 = 255;

int iLowV2 = 0;
int iHighV2 = 255;
/*New*/

Point red;
vector<Point> outlineRed;

Point white;
vector<Point> outlineWhite;

Point purple;
vector<Point> outlinePurple;

Point red2;
vector<Point> outlineRed2;

Point white2;
vector<Point> outlineWhite2;

Point purple2;
vector<Point> outlinePurple2;

vector<Mat> vector_mat(2);
Mat red_combined;
vector<Mat> views_vector(4);



Mat view, view2, view0, view1, imgThresholded, imgHSV, mask, orientation, previousFrame, previousFrame2;

int main(int argc, char* argv[])
{

	ISoundEngine* se = createIrrKlangDevice();

	VideoCapture capture, capture2;
	capture.open(0);
	capture2.open(1);
	if (!capture.isOpened())
	{
		int error = -1;
		return 1;
	}
	if (!capture2.isOpened())
	{
		int error = -1;
		return 1;
	}

	namedWindow("Views", CV_WINDOW_AUTOSIZE);
	resizeWindow("Views", 300, 600); // adjust trackbar width

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
	//createTrackbar("threshold", "Camera View", &smoothSlider, smoothSliderMax, onTrackbar);
	//createTrackbar("Blur", "Camera View", &smoothSlider, smoothSliderMax, onTrackbar2);
	createTrackbar("Color", "Views", &colorNumber, 6, onTrackbarSelect);
	createTrackbar("Red", "Views", &redThreshold, 255, onTrackbarRed);
	createTrackbar("Red2", "Views", &redThreshold2, 255, onTrackbarRed2);
	createTrackbar("Blue", "Views", &blueThreshold, 255, onTrackbarBlue);
	createTrackbar("Blue2", "Views", &blueThreshold2, 255, onTrackbarBlue2);
	

	//createTrackbar("yellow Threshold", "Views", &yellowThreshold, 255, onTrackbarYellow);
	

	//create a slider in the window2
	//createTrackbar("threshold", "Camera2 View", &smoothSlider, smoothSliderMax, onTrackbar);
	//createTrackbar("Blur", "Camera2 View", &smoothSlider, smoothSliderMax, onTrackbar2);
	
	
	//createTrackbar("yellow Threshold2", "Views", &yellowThreshold2, 255, onTrackbarYellow);
	//createTrackbar("Color", "Camera2 View", &colorNumber, 6, onTrackbarSelect);

	/*Trackers for HSV color detection*/
	
	cvCreateTrackbar("LowH", "Views", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Views", &iHighH, 179);

	cvCreateTrackbar("LowS", "Views", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Views", &iHighS, 255);

	cvCreateTrackbar("LowV", "Views", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Views", &iHighV, 255);
	
	cvCreateTrackbar("LowH2", "Views", &iLowH2, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH2", "Views", &iHighH2, 179);

	cvCreateTrackbar("LowS2", "Views", &iLowS2, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS2", "Views", &iHighS2, 255);

	cvCreateTrackbar("LowV2", "Views", &iLowV2, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV2", "Views", &iHighV2, 255);

	bool redStarted = false;

	int count = 0;

	
			
	while (capture.isOpened())
	{
		
		//grab and retrieve each frames of the video sequentially 
		//cv::Mat3b view0;
		capture >> view0;
		//cv::Mat3b frame1;
		capture2 >> view1;

		view2.copyTo(previousFrame2);// saves the previous frame
		capture2.read(view1);
		view1.copyTo(view2);
		cvtColor(view2, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		view.copyTo(previousFrame);// saves the previous frame
		capture.read(view0);
		view0.copyTo(view);
		cvtColor(view, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


		
		if (bRecording)
		{
			switch (colorNumber){
			
			case 0:
			
					  findLargestRedObject(view, red, outlineRed, redThreshold,0,true);
					  drawOutline(view0, outlineRed, 255, 0, 0);

					  findLargestRedObject2(view2, red2, outlineRed2, redThreshold2, 0, true);
					  drawOutline(view1, outlineRed2, 255, 0, 0);

				break;
			case 1:
			
					  setHSV(colorNumber);
					  findLargestGreenObject(view, green, outlineGreen, greenThreshold,1,true);
					  drawOutline(view0, outlineGreen, 0, 255, 0);

					  findLargestGreenObject2(view2, green2, outlineGreen2, greenThreshold2, 1, true);
					  drawOutline(view1, outlineGreen2, 0, 255, 0);

				break;
			case 2:
			
					  findLargestBlueObject(view, blue, outlineBlue, blueThreshold,2,true);
					  drawOutline(view0, outlineBlue, 0, 0, 255);

					  findLargestBlueObject2(view2, blue2, outlineBlue2, blueThreshold2, 2, true);
					  drawOutline(view1, outlineBlue2, 0, 0, 255);

				break;
			case 3:
			
					  setHSV(colorNumber);
					  findLargestPurpleObject(view, purple, outlinePurple, purpleThreshold,3,true);
					  drawOutline(view0, outlinePurple, 255, 0, 128);

					  findLargestPurpleObject(view2, purple2, outlinePurple2, purpleThreshold2, 3, true);
					  drawOutline(view1, outlinePurple2, 255, 0, 128);

				break;
			case 4:
			
					  setHSV(colorNumber);
					  findLargestYellowObject(view, yellow, outlineYellow, yellowThreshold,4,true);
					  drawOutline(view0, outlineYellow, 255, 255, 0);

					  findLargestYellowObject(view2, yellow2, outlineYellow2, yellowThreshold2, 4, true);
					  drawOutline(view1, outlineYellow2, 255, 255, 0);
			
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

					findLargestRedObject2(view2, red2, outlineRed2, redThreshold2, 0, false);
					findLargestPurpleObject(view2, purple2, outlinePurple2, purpleThreshold2, 3, false);
					findLargestBlueObject2(view2, blue2, outlineBlue2, blueThreshold2, 2, false);
					findLargestYellowObject(view2, yellow2, outlineYellow2, yellowThreshold2, 4, false);
					findLargestGreenObject(view2, green2, outlineGreen2, greenThreshold2, 1, false);
				}

				int percentAreaCoveredRed = 100 - ((currentRedArea * 100) / largestRedArea);
				int percentAreaCoveredRed2 = 100 - ((currentRedArea2 * 100) / largestRedArea2);
				
				//cout << "RED OFFSET: " << redPlayedOffset << endl;
				//cout << "BLUE OFFSET: " << bluePlayedOffset << endl;
				//cout << "covered area red! = " << percentAreaCoveredRed << endl;
				//cout << "covered area red2! = " << percentAreaCoveredRed2 << endl;

				string soundFile = "sound/snare_hit.wav";
				string soundFile2 = "sound/hihat_hit.wav";
				string soundFile3 = "sound/hihat_hit.wav";



				if (percentAreaCoveredRed >= 15 && percentAreaCoveredRed <= 70 && percentAreaCoveredRed2 >= 5 && percentAreaCoveredRed2 <= 70 && redPlayedOffset == 0){
					//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);
					se->play2D(soundFile.c_str());
					redPlayedOffset = 2;
					
					
				}
				else {
					if (redPlayedOffset < 0) {
						redPlayedOffset = 0;
					}
					else {
						redPlayedOffset = redPlayedOffset - 1;
					}
				}

				int percentAreaCoveredBlue = 100 - ((currentBlueArea * 100) / largestBlueArea);
				int percentAreaCoveredBlue2 = 100 - ((currentBlueArea2 * 100) / largestBlueArea2);
				if (percentAreaCoveredBlue >= 15 && percentAreaCoveredBlue <= 70 && percentAreaCoveredBlue2 >= 5 && percentAreaCoveredBlue2 <= 70 && bluePlayedOffset == 0){
					//PlaySound("sound/hihat_hit.wav", NULL, SND_ASYNC);
					se->play2D(soundFile2.c_str());
					bluePlayedOffset = 3;
				}
				else {
					if (bluePlayedOffset < 0) {
						bluePlayedOffset = 2;
					}
					else {
						bluePlayedOffset = bluePlayedOffset - 1;
					}
				}

				int percentAreaCoveredGreen = 100 - ((currentGreenArea * 100) / largestGreenArea);
				int percentAreaCoveredGreen2 = 100 - ((currentGreenArea2 * 100) / largestGreenArea2);

				cout << "covered area green! = " << percentAreaCoveredGreen << endl;
				cout << "covered area green2! = " << percentAreaCoveredGreen2 << endl;
				if (percentAreaCoveredGreen >= 15 && percentAreaCoveredGreen <= 70 && percentAreaCoveredGreen2 >= 5 && percentAreaCoveredGreen2 <= 70 && greenPlayedOffset == 0){
					//PlaySound("sound/hihat_hit.wav", NULL, SND_ASYNC);
					se->play2D(soundFile2.c_str());
					greenPlayedOffset = 3;
				}
				else {
					if (greenPlayedOffset < 0) {
						greenPlayedOffset = 2;
					}
					else {
						greenPlayedOffset = greenPlayedOffset - 1;
					}
				}

				count++;

				drawOutline(view0, outlineRed, 255,0,0);
				drawOutline(view1, outlineRed2, 255, 0, 0);

				break;
			}

			
			views_vector[0] = view0;
			cvtColor(vector_mat[0], views_vector[1], CV_GRAY2RGB);
			views_vector[2] = view1;
			cvtColor(vector_mat[1], views_vector[3], CV_GRAY2RGB);

			imshow("Views", makeCanvas(views_vector, 280, 1));
			//imshow("view1", view1);

			frameNumber++;
		}

		char key = waitKey(33);
		if (key == 'p')
		{

			sprintf_s(filename, "%s/frame_%04d.jpg", directory, frameNumber);

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
	hHigh[colorNumber] = iHighH;
	sHigh[colorNumber] = iHighS;
	vHigh[colorNumber] = iHighV;
	hLow[colorNumber] = iLowH;
	sLow[colorNumber] = iLowS;
	vLow[colorNumber] = iLowV;

	hHigh2[colorNumber] = iHighH2;
	sHigh2[colorNumber] = iHighS2;
	vHigh2[colorNumber] = iHighV2;
	hLow2[colorNumber] = iLowH2;
	sLow2[colorNumber] = iLowS2;
	vLow2[colorNumber] = iLowV2;
}

void setThresh(int colorNumber){
	switch (colorNumber){
	
	case 0 :
		largestRedArea = currentRedArea;
		largestRedArea2 = currentRedArea2;
		cout << "red set";
		break;
	case 1:
		largestGreenArea = currentGreenArea;
		largestGreenArea2 = currentGreenArea2;
		break;
	case 2: 
		largestBlueArea = currentBlueArea;
		largestBlueArea2 = currentBlueArea2;
		break;
	case 3:
		largestPurpleArea = currentPurpleArea;
		largestPurpleArea2 = currentPurpleArea2;
		break;
	case 4:
		largestYellowArea = currentYellowArea;
		largestYellowArea2 = currentYellowArea2;
	case 5 :
		//largestWhiteArea = currentWhiteArea;
		break;
	default:
		break;

	}
	
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

void onTrackbarRed2(int value, void* data)
{

	redThreshold2 = value;

}


void onTrackbarBlue(int value, void* data)
{

	blueThreshold = value;

}

void onTrackbarBlue2(int value, void* data)
{

	blueThreshold2 = value;

}

void onTrackbarYellow(int value, void* data)
{

	yellowThreshold = value;

}

void onTrackbarSelect(int colorNum, void* data)
{
	colorNumber = colorNumber; // check
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
		justRed[0].copyTo(vector_mat[0]);
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

bool findLargestRedObject2(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
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
		justRed[0].copyTo(vector_mat[1]);
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
			currentRedArea2 = largestArea;
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
		justBlue[0].copyTo(vector_mat[0]);
		//cv::imshow("Just Blue", justBlue[0]);
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

bool findLargestBlueObject2(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow)
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
	if (imshow){
		justBlue[0].copyTo(vector_mat[1]);
		//cv::imshow("Just Blue", justBlue[0]);
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
			currentBlueArea2 = largestArea;
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
		justGreen[0].copyTo(vector_mat[0]);
		//cv::imshow("Just Green", justGreen[0]);
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


bool findLargestGreenObject2(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
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
	inRange(YCrCb[0], Scalar(hLow2[colorNum], sLow2[colorNum], vLow2[colorNum]), Scalar(hHigh2[colorNum], sHigh2[colorNum], vHigh2[colorNum]), justGreen[0]); //Threshold the image
	//inRange(view, Scalar(47, 82, 0), Scalar(111, 255, 103), justGreen[0]); //Threshold the image lib
	//imshow("green before", justGreen[0]);
	equalizeHist(justGreen[0], justGreen[0]);
	//imshow("green after", justGreen[0]);
	threshold(justGreen[0], justGreen[0], 0, 255, THRESH_BINARY);
	//imshow("GREEN", justGreen[0]);
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	if (imshow){
		justGreen[0].copyTo(vector_mat[1]);
		//cv::imshow("Just Green", justGreen[0]);
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
			currentGreenArea2 = largestArea;
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

/**
* @brief makeCanvas Makes composite image from the given images
* @param vecMat Vector of Images.
* @param windowHeight The height of the new composite image to be formed.
* @param nRows Number of rows of images. (Number of columns will be calculated
*              depending on the value of total number of images).
* @return new composite image.
*/
cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows) {
	int N = vecMat.size();
	nRows = nRows > N ? N : nRows;
	int edgeThickness = 10;
	int imagesPerRow = ceil(double(N) / nRows);
	int resizeHeight = floor(2.0 * ((floor(double(windowHeight - edgeThickness) / nRows)) / 2.0)) - edgeThickness;
	int maxRowLength = 0;

	std::vector<int> resizeWidth;
	for (int i = 0; i < N;) {
		int thisRowLen = 0;
		for (int k = 0; k < imagesPerRow; k++) {
			double aspectRatio = double(vecMat[i].cols) / vecMat[i].rows;
			int temp = int(ceil(resizeHeight * aspectRatio));
			resizeWidth.push_back(temp);
			thisRowLen += temp;
			if (++i == N) break;
		}
		if ((thisRowLen + edgeThickness * (imagesPerRow + 1)) > maxRowLength) {
			maxRowLength = thisRowLen + edgeThickness * (imagesPerRow + 1);
		}
	}
	int windowWidth = maxRowLength;

	Mat canvasImage(windowHeight, windowWidth, CV_8UC3, Scalar(0, 0, 0));

	for (int k = 0, i = 0; i < nRows; i++) {
		int y = i * resizeHeight + (i + 1) * edgeThickness;
		int x_end = edgeThickness;
		for (int j = 0; j < imagesPerRow && k < N; k++, j++) {
			int x = x_end;
			cv::Rect roi(x, y, resizeWidth[k], resizeHeight);
			cv::Mat target_ROI = canvasImage(roi);
			cv::resize(vecMat[k], target_ROI, target_ROI.size());
			x_end += resizeWidth[k] + edgeThickness;
		}
	}
	
	return canvasImage;
}


