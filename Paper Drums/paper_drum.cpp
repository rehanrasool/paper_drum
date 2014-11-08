/**
Author : Omar Waheed, Rehan Rasool
test
*/

#include "stdafx.h"
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <time.h>
#include <deque>;

using namespace cv;
using namespace std;




void convertToGrayScale(Mat& frame); // converts the image to grayscale (1 channel)
void computeDifferenceOfFrames(Mat& currentFrame, Mat& previousFrame, Mat& differenceMatrix); // computes difference of current and previous frames
void onTrackbar(int value, void* data);
void onTrackbar2(int value, void* data);
void smoothImage(Mat& image, double sigma);
Mat diffMatrix;
int thresholdOfDiffMatrix = 30;
int blurQuantity = 1;

int main(int argc, char* argv[])
{
    VideoCapture capture;
    capture.open(0);
    if(!capture.isOpened())
    {
        int error = -1;
        return 1;
    }

    namedWindow( "Camera View", 1 );
    // http://docs.opencv.org/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html

	int smoothSlider;
	int smoothSliderMax = 100;

    Mat view;
	Mat previousFrame;
	
	Mat mask;
	Mat orientation;
	float timeStamp = 0;
	Mat motionHistory;
	Mat segMask;
	vector<Rect> boundingRects;
    bool blink = false;
	float MHI_DURATION = 2;
    bool bRecording = false;
    int frameNumber=0;
	bool horizontalMotion = true;
	bool verticalMotion = true;
	deque<String> historyQueue;
	time_t starttime;
	time_t differenceTime;
	time_t timer;
    
	char directory[128]={'\0'};
    directory[0]='.';


    if(argc > 1)
    {
        strcpy_s(directory, argv[1]);
    }
    char filename[256];
	starttime = time(&timer); // time starts here



	//create a slider in the window
	createTrackbar("threshold", "Camera View", &smoothSlider, smoothSliderMax, onTrackbar);
	createTrackbar("Blur", "Camera View", &smoothSlider, smoothSliderMax, onTrackbar2);

	// initialze queue with 5 values
	for (int i = 0; i < 4; i++){
		historyQueue.push_front("no output");
	}


	//all of the stuff gets computed in the onTrackbar function so that things get recomputed 
	//when you apply different levels of smoothing
	//Here, we call it manually for initialization
	onTrackbar(30,NULL);
	onTrackbar2(1, NULL);

    while( capture.isOpened() )
    {
        Mat view0;
		view.copyTo(previousFrame);// saves the previous frame
        capture.read(view0);
        view0.copyTo(view);
		motionHistory = cv::Mat::zeros(view.rows, view.cols, CV_32FC1);//
		mask = cv::Mat::zeros(view.rows, view.cols, CV_8UC1);
		orientation = cv::Mat::zeros(view.rows, view.cols, CV_32FC1);
        imshow("Camera View", view);

        if(bRecording)
        {

			differenceTime = starttime - time(&timer); // get time difference from start
			timeStamp = (float)abs(differenceTime % 100);

			sprintf_s(filename, "%s/frame_%04d.jpg", directory, frameNumber);
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
			frameNumber++;
        }

        char key = waitKey(33);
        if(key == 'p')
        {
			
            sprintf_s(filename, "%s/frame_%04d.jpg", directory, frameNumber);
			convertToGrayScale(view);
			computeDifferenceOfFrames(view, previousFrame, diffMatrix);
			
			imwrite(filename, diffMatrix);
            frameNumber++;
        }
        if(key == ' ')
        {
            bRecording = !bRecording;
        }
        if(key == 'q')
        {
            break;
        }
    }

    return 0;
}

// convert a frame to grayscale
void convertToGrayScale(Mat& frame){
	Mat tempGray;
	cvtColor(frame,tempGray,CV_BGR2GRAY);
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
