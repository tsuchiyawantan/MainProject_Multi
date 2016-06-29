#pragma once
#include <iostream>
#include <sstream>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <atlbase.h>

#ifdef USE_AUDIO
#include "WaveFile.h"
#endif /* USE_AUDIO */

using namespace std;

#define HIGHT 200
#define WIDTH 200

class Human{
private:
	int humanID;
	int depthMax;
	int depthMin;
public:


	Human(){
		init();
	}
	~Human(){}

	void setHumanID(int i){
		humanID = i;
	}
	void setDepthMax(int i){
		depthMax = i;
	}
	void setDepthMin(int i){
		depthMin = i;
	}
	void init(){
		setHumanID(-1);
		setDepthMax(-1);
		setDepthMin(INFINITE);
	}
	int getHumanID(){
		return this->humanID;
	}
	int getDepthMax(){
		return this->depthMax;
	}
	int getDepthMin(){
		return this->depthMin;
	}
};

class Depth : public KinectControl

{
	// ******** depth *******
private:
public:
	cv::Mat bodyDepthImage;
	cv::Mat normalizeDepthImage;
	cv::Mat contourImage;
	Human human[6];
	
	void findDepthMaxMin(Human &human, int depth){
		if (human.getDepthMax() < depth) human.setDepthMax(depth);
		if (human.getDepthMin() > depth) human.setDepthMin(depth);
	}
	void setBodyDepth(){
		updateDepthFrame();
		updateBodyIndexFrame();
		bodyDepthImage = cv::Mat(depthHeight, depthWidth, CV_16UC1);
		for (int i = 0; i < 6; i++) human[i].init();
		for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
			int y = i / bodyIndexWidth;
			int x = i % bodyIndexWidth;
			int id = bodyIndexBuffer[i];
			if (id == 255) {
				bodyDepthImage.at<UINT16>(y, x) = 65535;
			}
		else if (human[id].getHumanID() == -1){
				human[id].setHumanID(id);
			}
			else {
				bodyDepthImage.at<UINT16>(y, x) = depthBuffer[i];
				findDepthMaxMin(human[id], depthBuffer[i]);
			}
		}
	}
	void setNormalizeDepth(cv::Mat &srcImg){
		normalizeDepthImage = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);
		for (int i = 0; i < srcImg.rows*srcImg.cols; i++) {
			int y = i / srcImg.cols;
			int x = i % srcImg.cols;
			int id = bodyIndexBuffer[i];
			if (id == 255) normalizeDepthImage.at<UCHAR>(y, x) = 255;
			else {
				normalizeDepthImage.at<UCHAR>(y, x) = (255 * (srcImg.at<UINT16>(y, x) - human[id].getDepthMin())) / (human[id].getDepthMax() - human[id].getDepthMin());
			}
		}
	}
	void setContour(cv::Mat &srcImg){
		cv::Mat image2 = srcImg.clone();
		contourImage = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC1);
		//dilateÇ≈îíÇñcí£
		cv::dilate(image2, image2, cv::Mat(), cv::Point(-1, -1), 1);
		//erodeÇ≈îíÇèkè¨
		cv::erode(image2, image2, cv::Mat(), cv::Point(-1, -1), 3);
		cv::Canny(image2, contourImage, 20, 75);
		
	}
};