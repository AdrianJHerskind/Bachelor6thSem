#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <ros/console.h>
#include "davinci_imageProcessing/nextStitchPoint.h"

using namespace cv;

Mat cameraImage(0,0,CV_8UC3);
Mat liveImage;
double a, b;
double aPerp, bPerp;

bool done = false;
bool continueToNextPoint = true;

const int imageHeight = 576;
const int imageWidth = 720;

int nextStitchEntryX, nextStitchEntryY, nextStitchExitX, nextStitchExitY;

int averageArray[imageWidth][imageHeight][3];

/*
	Calculates next stitch point based on middlestitchpoint
	Goes along perpendicular and finds stitch point
*/
std::vector<Point> getNextStitchPoint(Point nextMiddleStitchPoint, Mat cannyImage)
{
	std::vector<Point> perpendicular;
	bPerp = nextMiddleStitchPoint.y - (aPerp*nextMiddleStitchPoint.x);
	for(int x = 0; x < cannyImage.cols; x++)
	{
		int y = aPerp*x + bPerp;
		if(y >= 0 && y < cannyImage.rows)
		{
			perpendicular.push_back(Point(x,y));
		}
	}
	int locationMiddleStitchPoint;
	bool firstPoint = true;
	bool secondPoint = false;
	Point firstPointPoint;
	Point secondPointPoint;
	for(int j = 0; j < perpendicular.size(); j++)
	{
		if(perpendicular.at(j).x == nextMiddleStitchPoint.x && perpendicular.at(j).y == nextMiddleStitchPoint.y)
		{
			locationMiddleStitchPoint = j;
		}
		Point KernelMiddle = perpendicular.at(j);
		Vec3b color0 = cannyImage.at<Vec3b>(Point(KernelMiddle.x-1, KernelMiddle.y-1));
		Vec3b color1 = cannyImage.at<Vec3b>(Point(KernelMiddle.x, KernelMiddle.y-1));
		Vec3b color2 = cannyImage.at<Vec3b>(Point(KernelMiddle.x+1, KernelMiddle.y-1));
		Vec3b color3 = cannyImage.at<Vec3b>(Point(KernelMiddle.x-1, KernelMiddle.y));
		Vec3b color4 = cannyImage.at<Vec3b>(Point(KernelMiddle.x, KernelMiddle.y));
		Vec3b color5 = cannyImage.at<Vec3b>(Point(KernelMiddle.x+1, KernelMiddle.y));
		Vec3b color6 = cannyImage.at<Vec3b>(Point(KernelMiddle.x-1, KernelMiddle.y+1));
		Vec3b color7 = cannyImage.at<Vec3b>(Point(KernelMiddle.x, KernelMiddle.y+1));
		Vec3b color8 = cannyImage.at<Vec3b>(Point(KernelMiddle.x+1, KernelMiddle.y+1));

		bool hit = false;
		if(color0[0] != 0 && color0[1] != 0 && color0[2] != 0 && color0[2] != 255)
		{
			hit = true;
		}
		else if(color1[0] != 0 && color1[1] != 0 && color1[2] != 0 && color1[2] != 255)
		{
			hit = true;
		}
		else if(color2[0] != 0 && color2[1] != 0 && color2[2] != 0 && color2[2] != 255)
		{
			hit = true;
		}
		else if(color3[0] != 0 && color3[1] != 0 && color3[2] != 0 && color3[2] != 255)
		{
			hit = true;
		}
		else if(color4[0] != 0 && color4[1] != 0 && color4[2] != 0 && color4[2] != 255)
		{
			hit = true;
		}
		else if(color5[0] != 0 && color5[1] != 0 && color5[2] != 0 && color5[2] != 255)
		{
			hit = true;
		}
		else if(color6[0] != 0 && color6[1] != 0 && color6[2] != 0 && color6[2] != 255)
		{
			hit = true;
		}
		else if(color7[0] != 0 && color7[1] != 0 && color7[2] != 0 && color7[2] != 255)
		{
			hit = true;
		}
		else if(color8[0] != 0 && color8[1] != 0 && color8[2] != 0 && color8[2] != 255)
		{
			hit = true;
		}
		if(hit)
		{
			color0[0] = 255;
			color0[1] = 0;
			color0[2] = 0;
			if(firstPoint)
			{
				if(j-10 > 0)
				{
					cannyImage.at<Vec3b>(perpendicular.at(j-10)) = color0;
					firstPointPoint = perpendicular.at(j-10);
					firstPoint = false;
					secondPoint = true;
				}
			}
			else if(secondPoint)
			{
				secondPoint = false;
				if(j+10 < perpendicular.size())
				{
					cannyImage.at<Vec3b>(perpendicular.at(j+14)) = color0;
					secondPointPoint = perpendicular.at(j+14);
				}
			}
		}
	}
	if(firstPoint == true)
	{
		firstPointPoint = perpendicular.at(locationMiddleStitchPoint-7);
	}	
	if(firstPoint == false && secondPoint == true)
	{
		secondPointPoint = perpendicular.at(locationMiddleStitchPoint+7);
	}
	//line(cannyImage, perpendicular.front(), perpendicular.back(), Scalar(0,255,0));
	imshow("viewCanny", cannyImage);

	std::vector<Point> bothPoints;
	bothPoints.push_back(firstPointPoint);
	bothPoints.push_back(secondPointPoint);
	return bothPoints;	
}

/*
	Returns next middleStitchPoint
*/
Point getNextMiddleStitchPoint(vector<Point> middleStitchPoints)
{
	Point nextMiddleStitchPoint = middleStitchPoints.back();
	return nextMiddleStitchPoint;
}

/*
	Calculates points on a line.
*/
std::vector<Point> getStitchMiddlePoints(std::vector<Point> line)
{
	int stitchDistanceInPixel = 20;
	std::vector<Point> stitchMiddlePoints;

	for(int i = 0; i < line.size(); i = i+stitchDistanceInPixel)
	{
		stitchMiddlePoints.push_back(Point(line.at(i)));
	}
	return stitchMiddlePoints;
}

/*
	Calculates a line from one point to the other, returns a vector with all points on the line
*/
std::vector<Point> calculateLine(Point lowerPoint, Point higherPoint) 
{
 	std::vector<Point> lineVector;
	bool isXLower;
	isXLower = (lowerPoint.x < higherPoint.x);

if(lowerPoint.y != 0 && higherPoint.y != 0 && lowerPoint.x != 0 && higherPoint.x != 0)
	{
		a = ((double)lowerPoint.y - (double)higherPoint.y)/((double)lowerPoint.x - (double)higherPoint.x+(higherPoint.x == lowerPoint.x));
		b = (lowerPoint.y - (a * lowerPoint.x));
		aPerp = -(1/a);
	}

	if(abs(lowerPoint.x - higherPoint.x) >= abs(lowerPoint.y - higherPoint.y))
	{
		if(lowerPoint.x < higherPoint.x)
		{
			for(int x = lowerPoint.x; x <= higherPoint.x; x++)
			{
				int y = (int)(a*x+b);
				lineVector.push_back(Point(x, y));
			}
		}
		else
		{
			for(int x = higherPoint.x; x <= lowerPoint.x; x++)
			{
				int y = (int)(a*x+b);
				lineVector.push_back(Point(x, y));
			}
		}
	}
	else
	{
		if(lowerPoint.y < higherPoint.y)
		{
			for(int y = lowerPoint.y; y <= higherPoint.y; y++)
			{
				int x = (int)((y-b)/a);
				lineVector.push_back(Point(x, y));
			}
		}
		else
		{
			for(int y = higherPoint.y; y <= lowerPoint.y; y++)
			{
				int x = (int)((y-b)/a);
				lineVector.push_back(Point(x, y));
			}
		}
	}
  	return lineVector;
}

/**
	Finds the line spanning from the extremes of the wound
*/
std::vector<Point> getExtremesLine(Mat cannyImage)
{
	int WoundMaxY[2] = {0,0};
	int WoundMaxX[2] = {0,0};
	int WoundMinY[2] = {0, cannyImage.rows-10};
	int WoundMinX[2] = {cannyImage.cols-10, 0};
	
	for(int i = 10; i < cannyImage.cols-10; i++)
	{
		for(int j = 10; j < cannyImage.rows-10; j++)
		{
			Vec3b color = cannyImage.at<Vec3b>(Point(i,j));
			if(color[0] > 0 && color[1] > 0 && color[2] > 0)
			{
				if(j > WoundMaxY[1]) {WoundMaxY[0] = i; WoundMaxY[1] = j;}
				if(i > WoundMaxX[0]) {WoundMaxX[0] = i; WoundMaxX[1] = j;}
				if(j < WoundMinY[1]) {WoundMinY[0] = i; WoundMinY[1] = j;}
				if(i < WoundMinX[0]) {WoundMinX[0] = i; WoundMinX[1] = j;}
			}
		}
	}
	Vec3b color;
	color[0] = 0; 
	color[1] = 0;
	color[2] = 255;
	Scalar red = Scalar(0,0,255);
	Point WoundMaxYPoint = Point(WoundMaxY[0], WoundMaxY[1]);
	Point WoundMaxXPoint = Point(WoundMaxX[0], WoundMaxX[1]);
	Point WoundMinYPoint = Point(WoundMinY[0], WoundMinY[1]);
	Point WoundMinXPoint = Point(WoundMinX[0], WoundMinX[1]);
	std::vector<Point> linePositions;
	if((WoundMaxYPoint.y - WoundMinYPoint.y) > (WoundMaxXPoint.x - WoundMinXPoint.x))
	{
		cannyImage.at<Vec3b>(WoundMaxYPoint) = color;
		cannyImage.at<Vec3b>(WoundMinYPoint) = color;
		circle(cannyImage, WoundMaxYPoint, 2, red, -1);
		circle(cannyImage, WoundMinYPoint, 2, red, -1);	
		linePositions = calculateLine(WoundMinYPoint, WoundMaxYPoint);
	}
	else
	{
		cannyImage.at<Vec3b>(WoundMaxXPoint) = color;
		cannyImage.at<Vec3b>(WoundMinXPoint) = color;
		circle(cannyImage, WoundMaxXPoint, 2, red, -1);
		circle(cannyImage, WoundMinXPoint, 2, red, -1);
		linePositions = calculateLine(WoundMinXPoint, WoundMaxXPoint);
	}

	if(!linePositions.empty())
	{
		line(cannyImage, linePositions.front(), linePositions.back(), red);
		imshow("viewExtremesLine", cannyImage);
	}
	return linePositions;
}

/**
	Canny Image
*/
Mat getCannyImage(Mat colorThresholdImage)
{
	Mat cannyInput(0,0,CV_8U);
	colorThresholdImage.copyTo(cannyInput);
	Mat imageWithCanny(0,0,CV_8U);
	int edgeThresh = 1;
	int lowThreshold = 100;
	int const max_lowThreshold = 100;
	int ratio = 4;
	int kernel_size = 3;
	Canny( cannyInput, imageWithCanny, lowThreshold, lowThreshold*ratio, kernel_size );
	Mat dst;	
	dst.create(cameraImage.size(), cameraImage.type() );
	dst = Scalar::all(0);
 	cameraImage.copyTo( dst, imageWithCanny);
	return dst;
}

/**
	Calculates the thresholded Color Image
*/
Mat getThresholdImage(Mat averagedImage)
{
	Mat colorThresholdImage;
	averagedImage.copyTo(colorThresholdImage);
	for(int i = 0; i < averagedImage.cols; i++)
	{
		for(int j = 0; j < averagedImage.rows; j++)
		{
			Vec3b color = averagedImage.at<Vec3b>(Point(i,j));
			if(abs(color[1] - color[2]) < 30 && color[1] > 51 && color[2] > 51 && color[0] < 75 && abs(color[1] - color[0]) > 50)
			{
			    	color[0] = 255;
				color[1] = 255;
				color[2] = 255;
			}
			else
			{
				color.val[0] = 1;
				color.val[1] = 0;
				color.val[2] = 0;
			}
			colorThresholdImage.at<Vec3b>(Point(i,j)) = color;
		}
	}
	//std::cout << "AND I PASSED IT ALL!" << "\n";
	//std::cout << colorThresholdImage.rows << "\n";
	return colorThresholdImage;
}

/**
	Adds up image values until 30 has been reached, then calculates average and returns averageImage
*/
Mat getAverageImage(int numberOfImagesRead)
{
	//std::cout << "numberOfImagesRead " << numberOfImagesRead << "\n";
	for(int i = 0; i < cameraImage.cols; i++)
	{
		for(int j = 0; j < cameraImage.rows; j++)
		{
			Vec3b color = cameraImage.at<Vec3b>(Point(i,j));
			averageArray[i][j][0] += color[0];
			averageArray[i][j][1] += color[1];
			averageArray[i][j][2] += color[2];
		}
	}

	if(numberOfImagesRead == 30)
	{
		Mat averagedImage;
		cameraImage.copyTo(averagedImage);
		for(int i = 0; i < cameraImage.cols; i++)
		{
			for(int j = 0; j < cameraImage.rows; j++)
			{
				Vec3b color;
				color[0] = (int)(averageArray[i][j][0] / 30.0);
				color[1] = (int)(averageArray[i][j][1] / 30.0);
				color[2] = (int)(averageArray[i][j][2] / 30.0);
				if(i < 20 || i > cameraImage.cols-20 || j < 20 || j > cameraImage.rows - 20)
				{
					color[1] = 0;
					color[2] = 0;
					color[0] = 0;
				}
				averagedImage.at<Vec3b>(Point(i,j)) = color;
				averageArray[i][j][0] = 0;
				averageArray[i][j][1] = 0;
				averageArray[i][j][2] = 0;
			}	
		}
		blur(averagedImage, averagedImage, Size(3,3));
		return averagedImage;
	}
	Mat emptyMat;
	return emptyMat;
}

/**
Receives the image and saves it in a variable
*/
void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg)
{
  	try
  	{
    	cv::imshow("viewRight", cv_bridge::toCvShare(msg, "bgr8")->image);
    	cv::waitKey(30);
  	}
  	catch (cv_bridge::Exception& e)
  	{
    		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
	cameraImage = cv_bridge::toCvShare(msg, "bgr8")->image;
	liveImage = cv_bridge::toCvShare(msg, "bgr8")->image;
}

/**
	Shows Live Image
*/
void showLiveImage()
{
	Mat superImposedStitchPoint(imageHeight, imageWidth, CV_8UC3);
	Mat superImposedStitchPointMask(imageHeight, imageWidth, CV_8UC3);
	for (int i = 0; i < superImposedStitchPointMask.cols; i++)
	{
		for(int j = 0; j < superImposedStitchPointMask.rows; j++)
		{
			Vec3b color = superImposedStitchPointMask.at<Vec3b>(Point(i,j));
			color[0] = 255; color[1] = 255; color[2] = 255;
			superImposedStitchPointMask.at<Vec3b>(Point(i,j)) = color;
		}
	}
	circle(superImposedStitchPointMask, Point(nextStitchEntryX, nextStitchEntryY), 3, Scalar(0,0,0), -1, 8);
	circle(superImposedStitchPointMask, Point(nextStitchEntryX, nextStitchEntryY), 2, Scalar(0,255,0), -1, 8);
	circle(superImposedStitchPointMask, Point(nextStitchExitX, nextStitchExitY), 3, Scalar(0,0,0), -1, 8);
	circle(superImposedStitchPointMask, Point(nextStitchExitX, nextStitchExitY), 2, Scalar(0,0,255), -1, 8);
	superImposedStitchPoint = Scalar::all(0);
	cameraImage.copyTo(superImposedStitchPoint, superImposedStitchPointMask);
	if(superImposedStitchPoint.rows != 0)
	{
		//imshow("viewSuperImposedStitchPoint", superImposedStitchPoint);
		if(cameraImage.rows != 0)
		{
			imshow("viewSuperImposedStitchPoint", superImposedStitchPoint);
		}
	}
}
/**
creates a service to to send back the coordinates of the next stitchpoints
*/
bool returnNextStitchPoint(davinci_imageProcessing::nextStitchPoint::Request  &req,
         davinci_imageProcessing::nextStitchPoint::Response &res)
{
	std::cout << "In Service nextStitchEntryX " << nextStitchEntryX << "\n";
	std::cout << "In Service nextStitchEntryY " << nextStitchEntryY << "\n";
	res.EntryX = nextStitchEntryX;
	res.EntryY = nextStitchEntryY;
	res.ExitX = nextStitchExitX;
	res.ExitY = nextStitchExitY;
	continueToNextPoint = true;
	//showLiveImage();
	ROS_INFO("sending back response: [%ld]", (long int)res.EntryX);
	ROS_INFO("sending back response: [%ld]", (long int)res.EntryY);
	ROS_INFO("sending back response: [%ld]", (long int)res.ExitX);
	ROS_INFO("sending back response: [%ld]", (long int)res.ExitY);
	return true;
}

/**
MAIN FUNCTION! Executes all the individual functions.
*/
int main(int argc, char **argv)
{
	//print the used opencv version
	//std:: cout<<"OpenCV Version used:" << CV_VERSION << "\n";
						
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	bool isInitialCalculated = false;

	//Average Variables
	bool isAverageImageCalculated = false;	
	int numberOfImagesRead = 0;
	Mat averagedImage;

	//Threshold Variables
	Mat colorThresholdImage(0,0, CV_8UC3);

	//Canny Variables
	Mat cannyImage;

	//Extremes Line Variables
	std::vector<Point> extremesLine;

	//MiddleStitchPoints Variables
	std::vector<Point> middleStitchPoints;

	//NextMiddleStitchPoint Variable
	Mat nextMiddleStitchPointImage;

	//Display the following images 
	namedWindow("viewRight", WINDOW_NORMAL);
	namedWindow("viewAverage", WINDOW_NORMAL);
	namedWindow("viewThreshold", WINDOW_NORMAL);
	namedWindow("viewCanny", WINDOW_NORMAL);
	namedWindow("viewExtremesLine", WINDOW_NORMAL);
	namedWindow("viewNextMiddleStitchPoint", WINDOW_NORMAL);
	namedWindow("viewSuperImposedStitchPoint", WINDOW_NORMAL);
	startWindowThread();
	image_transport::ImageTransport it(nh);
	std::string inputCameraPath = "/camera/rgb/image_raw";
	std::string inputEndoscope = "/endoscope/left/image_raw";
	image_transport::Subscriber subRight = it.subscribe(inputEndoscope, 1, imageCallbackRight);
	ros::ServiceServer service = nh.advertiseService("get_next_stitch_point", returnNextStitchPoint);
	ros::Rate r(100);
	while(ros::ok())
	{
		showLiveImage();
		if(!isInitialCalculated)
		{
			//If the average image has not yet been calculated
			if(!isAverageImageCalculated && cameraImage.rows != 0)
			{

				//1) Get AverageImage
				numberOfImagesRead++;
				averagedImage = getAverageImage(numberOfImagesRead);
				if(numberOfImagesRead == 30)
				{
					isAverageImageCalculated = true;
					numberOfImagesRead = 0;
					if(averagedImage.rows != 0)
					{
						imshow("viewAverage", averagedImage);
					}
				}
	
			}
			else
			{

				//1.5) GetColorThresholdImage
				colorThresholdImage = getThresholdImage(averagedImage);
				if(colorThresholdImage.rows != 0)
				{
					imshow("viewThreshold", colorThresholdImage);
				}

				//1.7) GetCannyImage
				if(colorThresholdImage.rows != 0)
				{
					cannyImage = getCannyImage(colorThresholdImage);
				}

				if(cannyImage.rows != 0)
				{				
					//imshow("viewCanny", cannyImage);
				}

				//2) Calculate Extremes Line (Which contains all points on the line from the left to the right of the wound
				if(cannyImage.rows != 0)
				{
					extremesLine = getExtremesLine(cannyImage);
				}

				//3) Calculate MiddleStitchPoints
				middleStitchPoints = getStitchMiddlePoints(extremesLine);
				if(!middleStitchPoints.empty())
				{
					isInitialCalculated = true;
					isAverageImageCalculated = false;
				}
			}
		}
		else
		{
			if(continueToNextPoint)
			{
				//1) Get AverageImage
				//If the average image has not yet been calculated
				if(!isAverageImageCalculated && cameraImage.rows != 0)
				{
					numberOfImagesRead++;
					averagedImage = getAverageImage(numberOfImagesRead);
					if(numberOfImagesRead == 30)
					{
						isAverageImageCalculated = true;
						numberOfImagesRead = 0;
						if(averagedImage.rows != 0)
						{
							imshow("viewAverage", averagedImage);
						}
					}
				}
				else
				{
					
					//1.5) GetColorThresholdImage
					colorThresholdImage = getThresholdImage(averagedImage);
					if(colorThresholdImage.rows != 0)
					{
						imshow("viewThreshold", colorThresholdImage);
					}

					//1.7) GetCannyImage
					if(colorThresholdImage.rows != 0)
					{
						cannyImage = getCannyImage(colorThresholdImage);
					}
					if(cannyImage.rows != 0)
					{				
						//imshow("viewCanny", cannyImage);
					}
					Point nextMiddleStitchPoint;

					//2) GoToNext MiddleStitchPoint
					if(!middleStitchPoints.empty())
					{
						nextMiddleStitchPoint = getNextMiddleStitchPoint(middleStitchPoints);
						middleStitchPoints.pop_back();
					}
					else
					{
						done = true;
					}

					//std::cout << "DO I GET HERE?";
					cannyImage.copyTo(nextMiddleStitchPointImage);
					Vec3b color;
					color[0] = 255;
					color[1] = 0;
					color[2] = 0;
					circle(nextMiddleStitchPointImage, nextMiddleStitchPoint, 2, Scalar(255,255,0), -1);
					imshow("viewNextMiddleStitchPoint", nextMiddleStitchPointImage);

					//3) Calculate NextStitchPoint
					std::vector<Point> nextStitchPoint;
					nextStitchPoint = getNextStitchPoint(nextMiddleStitchPoint, cannyImage);
					nextStitchEntryX = nextStitchPoint.at(0).x;
					nextStitchEntryY = nextStitchPoint.at(0).y;
					nextStitchExitX = nextStitchPoint.at(1).x;
					nextStitchExitY = nextStitchPoint.at(1).y;
					std::cout << "nextStitchPoint.x " << nextStitchPoint.at(0).x << "\n";
					std::cout << "nextStitchPoint.y " << nextStitchPoint.at(0).y << "\n";
					std::cout << "nextStitchEntryX " << nextStitchEntryX << "\n";
					std::cout << "nextStitchEntryY " << nextStitchEntryY << "\n"; 
					continueToNextPoint = false;
					if(done)
					{
						char input;
						std::cin >> input;
						ros::shutdown();
					}
				}
			}
		}
		r.sleep();
		ros::spinOnce();
	}
	cv::destroyWindow("viewRight");
	cv::destroyWindow("viewAverage");
	cv::destroyWindow("viewCanny");
	cv::destroyWindow("viewThreshold");
	cv::destroyWindow("viewExtremesLine");
	cv::destroyWindow("viewNextMiddleStitchPoint");
	cv::destroyWindow("viewSuperImposedStitchPoint");
}
