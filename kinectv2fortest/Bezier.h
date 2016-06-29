#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include "NeonDesign.h"

using namespace std;

class Bezier{
private:
public:
	Bezier(){}
	~Bezier(){}

	int checkVal(int n, int size){
		if (n < 0) return 0;
		else if (n >= size) return size - 1;
		return n;
	}
	void insertBezierPoints(pair<int, int> p2, pair<int, int> p3, vector<pair<int, int>> &yx, int i, cv::Mat &image){
		int newPY = checkVal(2 * p3.first - p2.first, image.rows);
		int newPX = checkVal(2 * p3.second - p2.second, image.cols);
		yx.insert(yx.begin() + i, make_pair(newPY, newPX));
	}
	void bezierLike(cv::Mat &srcImg, vector<vector<pair<int, int>>> &contours){
		for (int i = 0; i < contours.size(); i++){
			for (int j = 0; j < contours[i].size(); j += 3){
				if (j >= contours[i].size() || j + 1 >= contours[i].size() || j + 2 >= contours[i].size() || j + 3 >= contours[i].size()) break;
				pair<int, int> p2 = make_pair(contours[i].at(j + 2).first, contours[i].at(j + 2).second);
				pair<int, int> p3 = make_pair(contours[i].at(j + 3).first, contours[i].at(j + 3).second);
				insertBezierPoints(p2, p3, contours[i], j + 4, srcImg);
			}
		}
	}
	void adjust(vector<pair<int, int>> &yx, int i){
		int j = i;
		while (j < i + 4){
			if (j++ < yx.size()) continue;
			yx.push_back(make_pair(yx.back().first, yx.back().second));
		}
	}
	void drawInline(vector<vector<pair<int, int>>> &vec, cv::Mat &image, int hue){
		NeonDesign design;
		vector<int> bgr = { 0, 0, 0 };
		design.rgb(hue, 255 - 100, 255, bgr);
		for (int i = 0; i < vec.size(); i++){
			for (int j = 0; j < vec[i].size() - 1; j += 3){
				adjust(vec[i], j);
				if (j >= vec[i].size() || j + 1 >= vec[i].size() || j + 2 >= vec[i].size() || j + 3 >= vec[i].size()) break;
				for (double t = 0.0; t <= 1.0; t = t + 0.02){
					// 0.0 <= t && t <= 1.0 とする。この値を変化させて曲線を作る
					//3次のベジエ曲線
					int y = (1 - t)*(1 - t)*(1 - t)*vec[i].at(j).first + 3 * (1 - t)*(1 - t)*t*vec[i].at(j + 1).first + 3 * (1 - t)*t*t*vec[i].at(j + 2).first + t*t*t*vec[i].at(j + 3).first;
					int x = (1 - t)*(1 - t)*(1 - t)*vec[i].at(j).second + 3 * (1 - t)*(1 - t)*t*vec[i].at(j + 1).second + 3 * (1 - t)*t*t*vec[i].at(j + 2).second + t*t*t*vec[i].at(j + 3).second;
					circle(image, cv::Point(x, y), 1, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
				}
			}
		}
	}
	void drawBezier(vector<vector<pair<int, int>>> &forBezier, cv::Mat &image, int hue){
		NeonDesign design;
		vector<int> bgr = { 0, 0, 0 };
		design.rgb(hue, 255, 255 - 100, bgr);
		//cv::Mat bezierResult = cv::Mat(image.rows * SCALE, image.cols * SCALE, CV_8UC3, cv::Scalar(0, 0, 0));
		image = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0));
		for (int i = 0; i < forBezier.size(); i++){
			for (int j = 0; j < forBezier[i].size() - 1; j += 3){
				adjust(forBezier[i], j);
				if (j >= forBezier[i].size() || j + 1 >= forBezier[i].size() || j + 2 >= forBezier[i].size() || j + 3 >= forBezier[i].size()) break;
				for (double t = 0.0; t <= 1.0; t = t + 0.02){
					// 0.0 <= t && t <= 1.0 とする。この値を変化させて曲線を作る
					//3次のベジエ曲線
					int y = (1 - t)*(1 - t)*(1 - t)*forBezier[i].at(j).first + 3 * (1 - t)*(1 - t)*t*forBezier[i].at(j + 1).first + 3 * (1 - t)*t*t*forBezier[i].at(j + 2).first + t*t*t*forBezier[i].at(j + 3).first;
					int x = (1 - t)*(1 - t)*(1 - t)*forBezier[i].at(j).second + 3 * (1 - t)*(1 - t)*t*forBezier[i].at(j + 1).second + 3 * (1 - t)*t*t*forBezier[i].at(j + 2).second + t*t*t*forBezier[i].at(j + 3).second;
					circle(image, cv::Point(x, y), 5, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
					//image.at<cv::Vec3b>(y, x)[0] = 255; // b; //青
					//image.at<cv::Vec3b>(y, x)[1] = 255; // g; //緑
					//image.at<cv::Vec3b>(y, x)[2] = 255; // r; //赤
				}
			}
		}
		cv::GaussianBlur(image, image, cv::Size(19, 15), 0, 0);
		drawInline(forBezier, image, hue);
	}
};