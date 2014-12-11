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

Point blue;
vector<Point> outlineBlue;
Point blue2;
vector<Point> outlineBlue2;

Point green;
vector<Point> outlineGreen;
Point green2;
vector<Point> outlineGreen2;

Point purple;
vector<Point> outlinePurple;
Point purple2;
vector<Point> outlinePurple2;

Point brown;
vector<Point> outlineBrown;
Point brown2;
vector<Point> outlineBrown2;


Point red;
vector<Point> outlineRed;
Point red2;
vector<Point> outlineRed2;

/*new*/
cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows);
void computeObjectAreaAndCenter(vector<Point>& outline, double& area, Point& center);

Rect selection0[5];
Rect selection1[5];
int trackObject = 0;
bool selectObject = false;
Point origin;

void onMouse1(int event, int x, int y, int, void*);
void onMouse2(int event, int x, int y, int, void*);
bool findLargestRedObject(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestRedObject2(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestRedObjectTwo(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestRedObject2Two(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestRedObjectThree(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);
bool findLargestRedObject2Three(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow);

bool findLargestBlueObject(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum,bool imshow);
bool findLargestBlueObject2(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow);
bool findLargestBlueObjectTwo(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow);
bool findLargestBlueObject2Two(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow);

void drawOutline(Mat& image, vector<Point>& outline, int red, int green, int blue);
void setThresh(int colorNumber);

void onTrackbarRed(int value, void* data);
void onTrackbarRed2(int value, void* data);
void onTrackbarBlue(int value, void* data);
void onTrackbarBlue2(int value, void* data);
void onTrackbarRedTwo(int value, void* data);
void onTrackbarRed2Two(int value, void* data);
void onTrackbarBlueTwo(int value, void* data);
void onTrackbarBlue2Two(int value, void* data);
void onTrackbarRedThree(int value, void* data);
void onTrackbarRed2Three(int value, void* data);

void onTrackbarSelect(int colorNum, void* data);

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

int redThresholdTwo = 0;
int largestRedAreaTwo = 1;
int currentRedAreaTwo = 1;
int redThreshold2Two = 0;
int largestRedArea2Two = 1;
int currentRedArea2Two = 1;

int redThresholdThree = 0;
int largestRedAreaThree = 1;
int currentRedAreaThree = 1;
int redThreshold2Three = 0;
int largestRedArea2Three = 1;
int currentRedArea2Three = 1;


int blueThreshold = 1;
int largestBlueArea = 1;
int currentBlueArea = 1;
int blueThreshold2 = 1;
int largestBlueArea2 = 1;
int currentBlueArea2 = 1;

int blueThresholdTwo = 1;
int largestBlueAreaTwo = 1;
int currentBlueAreaTwo = 1;
int blueThreshold2Two = 1;
int largestBlueArea2Two = 1;
int currentBlueArea2Two = 1;

Mat justRedG;
Mat justGreenG;
Mat justPurpleG;
Mat justBrownG;
Mat justBlueG;

Mat justRedG2;
Mat justGreenG2;
Mat justPurpleG2;
Mat justBrownG2;
Mat justBlueG2;


//Store cropped images
Mat rois0[5];
Mat rois1[5];


vector<Mat> vector_mat(2); // stores masks
vector<Mat> views_vector(4); // stores views and masks

Mat view0, view1, imgThresholded, imgHSV, mask, orientation, previousFrame, previousFrame2;

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

	//namedWindow("Views", CV_WINDOW_AUTOSIZE);
	namedWindow("Views", CV_WINDOW_NORMAL);
	resizeWindow("Views", 400, 200); // adjust trackbar width

	int smoothSlider;
	int smoothSliderMax = 100;

	float MHI_DURATION = 2;
	bool bRecording = false;
	int frameNumber = 0;


	/*NEW*/
	
	/*NEW*/

	char directory[128] = { '\0' };
	directory[0] = '.';


	if (argc > 1)
	{
		strcpy_s(directory, argv[1]);
	}
	char filename[256];

	//create a slider in the window
	createTrackbar("Color", "Views", &colorNumber, 6, onTrackbarSelect);
	createTrackbar("Red", "Views", &redThreshold, 255, onTrackbarRed);
	createTrackbar("Red2", "Views", &redThreshold2, 255, onTrackbarRed2);
	createTrackbar("Blue", "Views", &blueThreshold, 255, onTrackbarBlue);
	createTrackbar("Blue2", "Views", &blueThreshold2, 255, onTrackbarBlue2);
	createTrackbar("RedTwo", "Views", &redThresholdTwo, 255, onTrackbarRed);
	createTrackbar("Red2Two", "Views", &redThreshold2Two, 255, onTrackbarRed2);
	createTrackbar("BlueTwo", "Views", &blueThresholdTwo, 255, onTrackbarBlue);
	createTrackbar("Blue2Two", "Views", &blueThreshold2Two, 255, onTrackbarBlue2);
	createTrackbar("RedThree", "Views", &redThresholdThree, 255, onTrackbarRed);
	createTrackbar("Red2Three", "Views", &redThreshold2Three, 255, onTrackbarRed2);

	int count = 0;
	Mat roi;
	Mat roiVisible;

	string soundRed = "sound/snare_hit.wav";
	string soundBlue = "sound/hihat_hit.wav";
	string soundGreen = "sound/tom1_hit.wav";
	string soundPurple = "sound/tom2_hit.wav";
	string soundBrown = "sound/Floortom_hit.wav";

	while (capture.isOpened())
	{
		
		//grab and retrieve each frames of the video sequentially 
		capture >> view0;
		capture2 >> view1;
		


		setMouseCallback("View 0", onMouse1, 0);
		setMouseCallback("View 1", onMouse2, 0);

		//if (roi.rows > 0){
			//imshow("ROI", rois[colorNumber]);
			//}
		capture2.read(view1);

		capture.read(view0);
		
		if (bRecording)
		{
			switch (colorNumber){
				
			case 0: // red
				
					roi = Mat(view0, selection0[colorNumber]);
					
					rois0[colorNumber] = roi;
					if (roi.rows > 0 && roi.rows > 0){
						imshow("roi", roi);
						findLargestRedObject(rois0[colorNumber], red, outlineRed, redThreshold, 0, true);
						drawOutline(rois0[colorNumber], outlineRed, 255, 0, 0);
					}
				
						
					roi = Mat(view1, selection1[colorNumber]);
					rois1[colorNumber] = roi;
					if (roi.rows > 0 && roi.rows > 0){
						findLargestRedObject2(rois1[colorNumber], red2, outlineRed2, redThreshold2, 0, true);
						drawOutline(rois1[colorNumber], outlineRed2, 255, 0, 0);

						int percentAreaCoveredRed = 100 - ((currentRedArea * 100) / largestRedArea);
						int percentAreaCoveredRed2 = 100 - ((currentRedArea2 * 100) / largestRedArea2);

						if (percentAreaCoveredRed >= 15 && percentAreaCoveredRed <= 70 && percentAreaCoveredRed2 >= 5 && percentAreaCoveredRed2 <= 70){
							//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

							se->play2D(soundRed.c_str());

						}


					}
				
					

				break;
			case 1: // blue
			
					roi = Mat(view0, selection0[colorNumber]);
					rois0[colorNumber] = roi;
					if (roi.rows > 0 && roi.cols > 0){
						findLargestBlueObject(rois0[colorNumber], blue, outlineBlue, blueThreshold, 2, true);
						drawOutline(rois0[colorNumber], outlineBlue, 0, 0, 255);
					}

					roi = Mat(view1, selection1[colorNumber]);
					rois1[colorNumber] = roi;
					if (roi.rows > 0 && roi.cols > 0){
						findLargestBlueObject2(rois1[colorNumber], blue2, outlineBlue2, blueThreshold2, 2, true);
						drawOutline(rois1[colorNumber], outlineBlue2, 0, 0, 255);

						int percentAreaCoveredBlue = 100 - ((currentBlueArea * 100) / largestBlueArea);
						int percentAreaCoveredBlue2 = 100 - ((currentBlueArea2 * 100) / largestBlueArea2);

						if (percentAreaCoveredBlue >= 15 && percentAreaCoveredBlue <= 70 && percentAreaCoveredBlue2 >= 5 && percentAreaCoveredBlue2 <= 70){
							//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

							se->play2D(soundBlue.c_str());


						}
					}


					  

				break;
			case 2: // red 2
					roi = Mat(view0, selection0[colorNumber]);

					rois0[colorNumber] = roi;
					if (roi.rows > 0 && roi.rows > 0){
						imshow("roi", roi);
						findLargestRedObjectTwo(rois0[colorNumber], red, outlineRed, redThresholdTwo, 0, true);
						drawOutline(rois0[colorNumber], outlineRed, 255, 0, 0);
					}


					roi = Mat(view1, selection1[colorNumber]);
					rois1[colorNumber] = roi;
					if (roi.rows > 0 && roi.rows > 0){
						findLargestRedObject2Two(rois1[colorNumber], red2, outlineRed2, redThreshold2Two, 0, true);
						drawOutline(rois1[colorNumber], outlineRed2, 255, 0, 0);

						int percentAreaCoveredRedTwo = 100 - ((currentRedAreaTwo * 100) / largestRedAreaTwo);
						int percentAreaCoveredRed2Two = 100 - ((currentRedArea2Two * 100) / largestRedArea2Two);

						if (percentAreaCoveredRedTwo >= 15 && percentAreaCoveredRedTwo <= 70 && percentAreaCoveredRed2Two >= 5 && percentAreaCoveredRed2Two <= 70){
							//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

							se->play2D(soundGreen.c_str());

						}


					}



					
				break;
			case 3: // blue 2
			
					roi = Mat(view0, selection0[colorNumber]);
					rois0[colorNumber] = roi;
					if (roi.rows > 0 && roi.cols > 0){
						findLargestBlueObjectTwo(rois0[colorNumber], blue, outlineBlue, blueThresholdTwo, 2, true);
						drawOutline(rois0[colorNumber], outlineBlue, 0, 0, 255);
					}

					roi = Mat(view1, selection1[colorNumber]);
					rois1[colorNumber] = roi;
					if (roi.rows > 0 && roi.cols > 0){
						findLargestBlueObject2Two(rois1[colorNumber], blue2, outlineBlue2, blueThreshold2Two, 2, true);
						drawOutline(rois1[colorNumber], outlineBlue2, 0, 0, 255);

						int percentAreaCoveredBlueTwo = 100 - ((currentBlueAreaTwo * 100) / largestBlueAreaTwo);
						int percentAreaCoveredBlue2Two = 100 - ((currentBlueArea2Two * 100) / largestBlueArea2Two);

						if (percentAreaCoveredBlueTwo >= 15 && percentAreaCoveredBlueTwo <= 70 && percentAreaCoveredBlue2Two >= 5 && percentAreaCoveredBlue2Two <= 70){
							//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

							se->play2D(soundPurple.c_str());


						}


				}

				break;
			case 4:// red 3
			
					roi = Mat(view0, selection0[colorNumber]);

					rois0[colorNumber] = roi;
					if (roi.rows > 0 && roi.rows > 0){
						imshow("roi", roi);
						findLargestRedObjectThree(rois0[colorNumber], red, outlineRed, redThresholdThree, 0, true);
						drawOutline(rois0[colorNumber], outlineRed, 255, 0, 0);
					}


					roi = Mat(view1, selection1[colorNumber]);
					rois1[colorNumber] = roi;
					if (roi.rows > 0 && roi.rows > 0){
						findLargestRedObject2Three(rois1[colorNumber], red2, outlineRed2, redThreshold2Three, 0, true);
						drawOutline(rois1[colorNumber], outlineRed2, 255, 0, 0);

						int percentAreaCoveredRedThree = 100 - ((currentRedAreaThree * 100) / largestRedAreaThree);
						int percentAreaCoveredRed2Three = 100 - ((currentRedArea2Three * 100) / largestRedArea2Three);

						if (percentAreaCoveredRedThree >= 15 && percentAreaCoveredRedThree <= 70 && percentAreaCoveredRed2Three >= 5 && percentAreaCoveredRed2Three <= 70){
							//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

							se->play2D(soundBrown.c_str());

						}


					}

				break;
			case 5:
				break;
			default:
					findLargestRedObject(rois0[0], red, outlineRed, redThreshold, 0, false);
					findLargestBlueObject(rois0[1], blue, outlineBlue, blueThreshold, 1, false);
					findLargestRedObjectTwo(rois0[2], red, outlineRed, redThresholdTwo, 2, false);
					findLargestBlueObjectTwo(rois0[3], blue, outlineBlue, blueThresholdTwo, 3, false);
					findLargestRedObjectThree(rois0[4], red, outlineRed, redThresholdThree, 4, false);

					findLargestRedObject2(rois1[0], red2, outlineRed2, redThreshold2, 0, false);
					findLargestBlueObject2(rois1[1], blue2, outlineBlue2, blueThreshold2, 1, false);
					findLargestRedObject2Two(rois1[2], red2, outlineRed2, redThreshold2Two, 2, false);
					findLargestBlueObject2Two(rois1[3], blue2, outlineBlue2, blueThreshold2Two, 3, false);
					findLargestRedObject2Three(rois1[4], red2, outlineRed2, redThreshold2Three, 4, false);


					int percentAreaCoveredRed = 100 - ((currentRedArea * 100) / largestRedArea);
					int percentAreaCoveredRed2 = 100 - ((currentRedArea2 * 100) / largestRedArea2);

					if (percentAreaCoveredRed >= 15 && percentAreaCoveredRed <= 70 && percentAreaCoveredRed2 >= 5 && percentAreaCoveredRed2 <= 70){
						//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

						se->play2D(soundRed.c_str());

					}


					int percentAreaCoveredBlue = 100 - ((currentBlueArea * 100) / largestBlueArea);
					int percentAreaCoveredBlue2 = 100 - ((currentBlueArea2 * 100) / largestBlueArea2);

					if (percentAreaCoveredBlue >= 15 && percentAreaCoveredBlue <= 70 && percentAreaCoveredBlue2 >= 5 && percentAreaCoveredBlue2 <= 70){
						//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

						se->play2D(soundBlue.c_str());


					}


					int percentAreaCoveredRedTwo = 100 - ((currentRedAreaTwo * 100) / largestRedAreaTwo);
					int percentAreaCoveredRed2Two = 100 - ((currentRedArea2Two * 100) / largestRedArea2Two);

					if (percentAreaCoveredRedTwo >= 15 && percentAreaCoveredRedTwo <= 70 && percentAreaCoveredRed2Two >= 5 && percentAreaCoveredRed2Two <= 70){
						//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

						se->play2D(soundGreen.c_str());

					}


					int percentAreaCoveredBlueTwo = 100 - ((currentBlueAreaTwo * 100) / largestBlueAreaTwo);
					int percentAreaCoveredBlue2Two = 100 - ((currentBlueArea2Two * 100) / largestBlueArea2Two);

					if (percentAreaCoveredBlueTwo >= 15 && percentAreaCoveredBlueTwo <= 70 && percentAreaCoveredBlue2Two >= 5 && percentAreaCoveredBlue2Two <= 70){
						//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

						se->play2D(soundPurple.c_str());


					}

					int percentAreaCoveredRedThree = 100 - ((currentRedAreaThree * 100) / largestRedAreaThree);
					int percentAreaCoveredRed2Three = 100 - ((currentRedArea2Three * 100) / largestRedArea2Three);

					if (percentAreaCoveredRedThree >= 15 && percentAreaCoveredRedThree <= 70 && percentAreaCoveredRed2Three >= 5 && percentAreaCoveredRed2Three <= 70){
						//PlaySound("sound/snare_hit.wav", NULL, SND_ASYNC);

						se->play2D(soundBrown.c_str());

					}


				count++;

				break;
			}

			if (rois0[colorNumber].rows > 0 && rois1[colorNumber].rows>0 ){
				views_vector[0] = rois0[colorNumber];
				cvtColor(vector_mat[0], views_vector[1], CV_GRAY2RGB);
				views_vector[2] = rois1[colorNumber];
				cvtColor(vector_mat[1], views_vector[3], CV_GRAY2RGB);

				imshow("Views", makeCanvas(views_vector, 100, 2));
				
			}
			
			frameNumber++;
		}
		imshow("View 0", view0);
		imshow("View 1", view1);

		char key = waitKey(33);
		if (key == 'r')
		{
			resizeWindow("Views", 900, 300); // adjust trackbar width

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

static void onMouse1(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection0[colorNumber].x = MIN(x, origin.x);
		selection0[colorNumber].y = MIN(y, origin.y);
		selection0[colorNumber].width = std::abs(x - origin.x);
		selection0[colorNumber].height = std::abs(y - origin.y);

		selection0[colorNumber] &= Rect(0, 0, view0.cols, view0.rows);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		origin = Point(x, y);		 
		selection0[colorNumber] = Rect(x, y, 100, 100);
		selectObject = true;
		break;
	case EVENT_LBUTTONUP:
		selectObject = false;
		if (selection0[colorNumber].width > 0 && selection0[colorNumber].height > 0)
			trackObject = -1;
		break;
	}
}

static void onMouse2(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection1[colorNumber].x = MIN(x, origin.x);
		selection1[colorNumber].y = MIN(y, origin.y);
		selection1[colorNumber].width = std::abs(x - origin.x);
		selection1[colorNumber].height = std::abs(y - origin.y);

		selection1[colorNumber] &= Rect(0, 0, view0.cols, view0.rows);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		origin = Point(x, y);
		selection1[colorNumber] = Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case EVENT_LBUTTONUP:
		selectObject = false;
		if (selection1[colorNumber].width > 0 && selection1[colorNumber].height > 0)
			trackObject = -1;
		break;
	}
}

void setThresh(int colorNumber){
	switch (colorNumber){
	
	case 0 :
		largestRedArea = currentRedArea;
		largestRedArea2 = currentRedArea2;
		cout << "red set";
		break;
	case 1:
		//largestGreenArea = currentGreenArea;
		//largestGreenArea2 = currentGreenArea2;
		largestBlueArea = currentBlueArea;
		largestBlueArea2 = currentBlueArea2;
		cout << "blue set";
		break;
	case 2: 
		//largestBlueArea = currentBlueArea;
		//largestBlueArea2 = currentBlueArea2;
		largestRedAreaTwo = currentRedArea;
		largestRedArea2Two = currentRedArea2;
		cout << "red 2 set";
		break;
	case 3:
		//largestPurpleArea = currentPurpleArea;
		//largestPurpleArea2 = currentPurpleArea2;
		largestBlueAreaTwo = currentBlueAreaTwo;
		largestBlueArea2Two = currentBlueArea2Two;
		cout << "blue 2 set";
		break;
	case 4:
		//largestBrownArea = currentBrownArea;
		//largestBrownArea2 = currentBrownArea2;
		largestRedAreaThree = currentRedAreaThree;
		largestRedArea2Three = currentRedArea2Three;
		cout << "red 3 set";
		break;
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

void onTrackbarRedTwo(int value, void* data)
{

	redThresholdTwo = value;

}

void onTrackbarRed2Two(int value, void* data)
{

	redThreshold2Two = value;

}

void onTrackbarRedThree(int value, void* data)
{

	redThresholdThree = value;

}

void onTrackbarRed2Three(int value, void* data)
{

	redThreshold2Three = value;

}


void onTrackbarBlue(int value, void* data)
{

	blueThreshold = value;

}

void onTrackbarBlue2(int value, void* data)
{

	blueThreshold2 = value;

}

void onTrackbarBlueTwo(int value, void* data)
{

	blueThreshold = value;

}

void onTrackbarBlue2Two(int value, void* data)
{

	blueThreshold2 = value;

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


void computeObjectAreaAndCenter(vector<Point>& outline, double& area, Point& center)
{
	// http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
	Moments objectProperties;
	objectProperties = moments(outline, false);

	area = objectProperties.m00;
	center.x = (objectProperties.m10 / area);
	center.y = (objectProperties.m01 / area);
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
	/*for (int i = 0; i < 1; i++){
		erode(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
		dilate(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
	if (imshow){
		justRed[0].copyTo(vector_mat[0]);
	}

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
	/*
	for (int i = 0; i < 1; i++){
		erode(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
		dilate(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	if (imshow){
		justRed[0].copyTo(vector_mat[1]);
	}

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

bool findLargestRedObjectTwo(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
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
	/*for (int i = 0; i < 1; i++){
	erode(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	dilate(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
	if (imshow){
		justRed[0].copyTo(vector_mat[0]);
	}

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
			currentRedAreaTwo = largestArea;
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

bool findLargestRedObject2Two(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
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
	/*
	for (int i = 0; i < 1; i++){
	erode(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	dilate(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	if (imshow){
		justRed[0].copyTo(vector_mat[1]);
	}

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
			currentRedArea2Two = largestArea;
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

bool findLargestRedObjectThree(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
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
	/*for (int i = 0; i < 1; i++){
	erode(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	dilate(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
	if (imshow){
		justRed[0].copyTo(vector_mat[0]);
	}

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
			currentRedAreaThree = largestArea;
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

bool findLargestRedObject2Three(Mat& view, Point& location, vector<Point>& outline, int redThreshold, int colorNum, bool imshow)
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
	/*
	for (int i = 0; i < 1; i++){
	erode(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	dilate(justRed[0], justRed[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	if (imshow){
		justRed[0].copyTo(vector_mat[1]);
	}

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
			currentRedArea2Three = largestArea;
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
	/*
	for (int i = 0; i < 1; i++){
		erode(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
		dilate(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
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
	/*
	for (int i = 0; i < 1; i++){
		erode(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
		dilate(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
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

bool findLargestBlueObjectTwo(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow)
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
	/*
	for (int i = 0; i < 1; i++){
	erode(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
	dilate(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
	vector<vector<Point>> objectContours;
	vector<Vec4i> dummy;
	if (imshow){
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
			currentBlueAreaTwo = largestArea;
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

bool findLargestBlueObject2Two(Mat& view, Point& location, vector<Point>& outline, int blueThreshold, int colorNum, bool imshow)
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
	/*
	for (int i = 0; i < 1; i++){
	erode(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
	dilate(justBlue[0], justBlue[0], Mat(), Point(-1, -1), 2, 1, 1);
	}
	*/
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
			currentBlueArea2Two = largestArea;
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


