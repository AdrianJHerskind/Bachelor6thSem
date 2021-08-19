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
Mat averageImage2;
double a, b;
double aPerp, bPerp;
double largestNumber = 0;
int largestNumber2 = 0;
double divider = 0;
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
	std::vector<Point> perpendicularx;
	std::vector<Point> perpendiculary;
	std::vector<Point> perpendicular;
	std::vector<Point> perpendicularTemp;
	bPerp = nextMiddleStitchPoint.y - (aPerp*nextMiddleStitchPoint.x);
	for(int x = 0; x < cannyImage.cols; x++)
	{
		int y = aPerp*(double)x + bPerp;
		if(y >= 0 && y < cannyImage.rows)
		{
			perpendicularx.push_back(Point(x,y));
		}
	}
	for(int y = 0; y < cannyImage.rows; y++)
	{
		int x = ((double)y-bPerp)/aPerp;
		if(x >= 0 && x < cannyImage.cols)
		{
			perpendiculary.push_back(Point(x,y));
		}
	}
	if(perpendicularx.size() > perpendiculary.size())
	{
		perpendicular = perpendicularx;
	}
	else
	{
		perpendicular = perpendiculary;
	}
	int pixCounter = 0;
	divider = perpendicular.size();
	for(int l = 0; l < perpendicular.size(); l++)
	{
		Vec3b color0 = cannyImage.at<Vec3b>(Point(perpendicular.at(l)));
		if(color0[0] == 255 && color0[1] == 255 && color0[2] == 255)
		{
		pixCounter++;
		}
	}
	if(pixCounter > largestNumber2)
	{
		largestNumber2 = pixCounter;
	}
	largestNumber += pixCounter;



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
		
		if(color4[0] == 255 && color4[1] == 255 && color4[2] == 255)
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
				if(j-45 > 0)
				{
					cannyImage.at<Vec3b>(perpendicular.at(j)) = color0;
					firstPointPoint = perpendicular.at(j);
					firstPoint = false;
					secondPoint = true;
				}
			}
			else if(secondPoint)
			{
				secondPoint = false;
				if(j+45 < perpendicular.size())
				{
					cannyImage.at<Vec3b>(perpendicular.at(j)) = color0;
					secondPointPoint = perpendicular.at(j);
				}
			}
		}
	}

	if(firstPoint == true && perpendicular.size() > 6 && locationMiddleStitchPoint >= 7)
	{
		firstPointPoint = perpendicular.at(locationMiddleStitchPoint-7);
	}	
	if(firstPoint == false && secondPoint == true && perpendicular.size() >= locationMiddleStitchPoint+7 && locationMiddleStitchPoint+7 <= perpendicular.size()-1)
	{
		secondPointPoint = perpendicular.at(locationMiddleStitchPoint+7);
	}


	line(cannyImage, perpendicular.front(), perpendicular.back(), Scalar(0,255,0));


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
	int stitchDistanceInPixel = 1;
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

	int WoundMaxY[2] = {40,40};
	int WoundMaxX[2] = {40,40};
	int WoundMinY[2] = {40, cannyImage.rows-40};
	int WoundMinX[2] = {cannyImage.cols-40, 40};
	
	for(int i = 40; i < cannyImage.cols-40; i++)
	{
		for(int j = 40; j < cannyImage.rows-40; j++)
		{
			Vec3b color = cannyImage.at<Vec3b>(Point(i,j));
			if(color[0] > 10 && color[1] > 10 && color[2] > 10)
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
			//if(color[0] > 250 && color[1] > 250 && color[2] > 250)
			if(color[0] < 100 && color[1] < 100 && color[2] > 200)
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
	return colorThresholdImage;
}

/**
	Adds up image values until 30 has been reached, then calculates average and returns averageImage
*/
Mat getAverageImage(int numberOfImagesRead)
{
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

	if(numberOfImagesRead == 1)
	{
		Mat averagedImage;
		cameraImage.copyTo(averagedImage);
		for(int i = 0; i < cameraImage.cols; i++)
		{
			for(int j = 0; j < cameraImage.rows; j++)
			{
				Vec3b color;
				color[0] = (int)(averageArray[i][j][0] / 1.0);
				color[1] = (int)(averageArray[i][j][1] / 1.0);
				color[2] = (int)(averageArray[i][j][2] / 1.0);
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
		//blur(averagedImage, averagedImage, Size(3,3));
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

		if(cameraImage.rows != 0)
		{
			Vec3b color;
			color[0] = 0; color[1] = 0; color[2] = 255;
			circle(superImposedStitchPoint, Point(109,242), 3, Scalar(255,255,0), 1);
			circle(superImposedStitchPoint, Point(107,372), 3, Scalar(255,255,0), 1);
			circle(superImposedStitchPoint, Point(502,185), 3, Scalar(255,255,0), 1);
			circle(superImposedStitchPoint, Point(506,384), 3, Scalar(255,255,0), 1);

		}
	}
}
/**
creates a service to to send back thhttps://mail.aau.dk/owa/auth/logon.aspx?replaceCurrent=1&url=https%3a%2f%2fmail.aau.dk%2fowa%2fe coordinates of the next stitchpoints
*/
bool returnNextStitchPoint(davinci_imageProcessing::nextStitchPoint::Request  &req,
         davinci_imageProcessing::nextStitchPoint::Response &res)
{
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


	startWindowThread();
	image_transport::ImageTransport it(nh);
	std::string inputCameraPath = "/camera/rgb/image_raw";
	std::string inputEndoscope = "/endoscope/left/image_raw";
	image_transport::Subscriber subRight = it.subscribe(inputEndoscope, 1, imageCallbackRight);
	ros::ServiceServer service = nh.advertiseService("get_next_stitch_point", returnNextStitchPoint);
	String path;
	String var;
	int counter;
	while(ros::ok())
	{
		double alpha = 0;
		double moveTime = 0;
		for(double i = 0.0; i <= 1.0; i = i + 0.01)
	{
		moveTime = moveTime+0.01;
		if(moveTime >= 0.0 && moveTime < 0.2)
		{
			alpha = 3.125 * (moveTime*moveTime);
		}
		else if(moveTime >= 0.2 && moveTime <= 0.8)
		{
			alpha = 1.25*moveTime-0.125;
		}
		else if(moveTime > 0.8 && moveTime <= 1.0)
		{
			alpha = -3.125 * (moveTime-1)*(moveTime-1)+1.0;
		}
		std::cout << "alpha " << alpha << "\n";
		std::cout << "movetime " << moveTime << "\n";
	}
	
		counter = 0;
		std::cin >> var;
		path = "/home/gr662/Desktop/662SVN/Images/Config4/" + var + " copy.jpg";
		std::cout << "path " << path << "\n";
		cameraImage = imread(path, CV_LOAD_IMAGE_COLOR);
		

		showLiveImage();


				//1) Get AverageImage
				numberOfImagesRead++;
				averagedImage = getAverageImage(numberOfImagesRead);
				if(numberOfImagesRead == 1)
				{
					isAverageImageCalculated = true;
					numberOfImagesRead = 0;
					if(averagedImage.rows != 0)
					{

					}
				}
	


				//1.5) GetColorThresholdImage
				colorThresholdImage = getThresholdImage(averagedImage);
				if(colorThresholdImage.rows != 0)
				{

				}
https://mail.aau.dk/owa/auth/logon.aspx?replaceCurrent=1&url=https%3a%2f%2fmail.aau.dk%2fowa%2f
				//1.7) GetCannyImage
				if(colorThresholdImage.rows != 0)
				{

				}

				if(cannyImage.rows != 0)
				{				

				}

				//2) Calculate Extremes Line (Which contains all points on the line from the left to the right of the wound
				if(colorThresholdImage.rows != 0)
				{
					extremesLine = getExtremesLine(colorThresholdImage);
				}

				//3) Calculate MiddleStitchPoints
				middleStitchPoints = getStitchMiddlePoints(extremesLine);
				if(!middleStitchPoints.empty())
				{
					isInitialCalculated = true;
					isAverageImageCalculated = false;
				}
		for(int i = 0; i < extremesLine.size(); i++)
		{
				//1) Get AverageImage
				//If the average image has not yet been calculated
				if(!isAverageImageCalculated && cameraImage.rows != 0)
				{
					numberOfImagesRead++;
					averagedImage = getAverageImage(numberOfImagesRead);
					if(numberOfImagesRead == 1)
					{
						isAverageImageCalculated = true;
						numberOfImagesRead = 0;
						if(averagedImage.rows != 0)
						{

						}
					}
				}

					//1.5) GetColorThresholdImage
					colorThresholdImage = getThresholdImage(averagedImage);
					if(colorThresholdImage.rows != 0)
					{

					}
					//1.7) GetCannyImage
					if(colorThresholdImage.rows != 0)
					{

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
					colorThresholdImage.copyTo(nextMiddleStitchPointImage);
					Vec3b color;
					color[0] = 255;
					color[1] = 0;
					color[2] = 0;
					circle(nextMiddleStitchPointImage, nextMiddleStitchPoint, 2, Scalar(255,255,0), -1);

					//3) Calculate NextStitchPoint
					std::vector<Point> nextStitchPoint;
					nextStitchPoint = getNextStitchPoint(nextMiddleStitchPoint, colorThresholdImage);
					nextStitchEntryX = nextStitchPoint.at(0).x;
					nextStitchEntryY = nextStitchPoint.at(0).y;
					nextStitchExitX = nextStitchPoint.at(1).x;
					nextStitchExitY = nextStitchPoint.at(1).y;

					continueToNextPoint = false;
					isAverageImageCalculated = false;
					if(done)
					{
						char input;
						std::cin >> input;
						ros::shutdown();
					}
			}
			std::cout << "DONE" << "\n";
			largestNumber = largestNumber / divider;
			std::cout << "averageBreadth: " << largestNumber << "\n";
			std::cout << "MaxHeight: " << largestNumber2 << "\n";
			largestNumber = 0;
			largestNumber2 = 0;
			divider = 0;	

		ros::spinOnce();
	}


}
