#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#include "ExecuteSpaceFiltering.h"
#include "NeonDesign.h"

using namespace std;

class CatmullSpline{
private:
public:
	vector<pair<int, int>> contour;
	vector<vector<pair<int, int>>> catmullLine;
	cv::Mat resultImg;

	CatmullSpline(){}
	~CatmullSpline(){}

	double catmullRom(double p0, double p1, double p2, double p3, double t){
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p0)*t + (2 * p0 - 5 * p1 + 4 * p2 - p3)*t2 + (-p0 + 3 * p1 - 3 * p2 + p3)*t3);
	}
	double catmullRomFirstLast(double p1, double p2, double t){
		double t2 = t*t;
		double t3 = t2*t;
		return 0.5*((2 * p1) + (p2 - p1)*t + (2 * p1 - 5 * p1 + 4 * p2 - p2)*t2 + (-p1 + 3 * p1 - 3 * p2 + p2)*t3);
	}
	void adjust(vector<pair<int, int>> &yx){
		int j = yx.size() + (4 - yx.size() % 4);
		while (yx.size() < j){
			yx.push_back(make_pair(yx.back().first, yx.back().second));
		}
	}

	boolean check8(cv::Mat& srcImg, int y, int x) {
		int n[8][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 } };
		int count = 0;
		for (int i = 0; i < 8; i++) {
			int dy = y + n[i][0];
			int dx = x + n[i][1];
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols) continue;
			if (srcImg.at<cv::Vec3b>(dy, dx)[0] != 255 &&
				srcImg.at<cv::Vec3b>(dy, dx)[1] != 255 &&
				srcImg.at<cv::Vec3b>(dy, dx)[2] != 255) return true;
		}
		return false;
	}
	void exeGaussian(vector<vector<pair<int, int>>> &vec, cv::Mat &srcImg, int filtersize){
		ExecuteSpaceFiltering spaceFilter(filtersize);
		resultImg = cv::Mat(srcImg.rows, srcImg.cols, CV_8UC3, cv::Scalar(255, 255, 255));

		for (int y = 0; y < srcImg.rows; y++){
			for (int x = 0; x < srcImg.cols; x++){
				if (check8(srcImg, y, x)) {
					spaceFilter.executeSpaceFilteringYX(y, x, srcImg, resultImg);
				}
			}
		}
		srcImg = resultImg;
	}

	void drawInline(cv::Mat &srcImg, int hue){
		NeonDesign design;
		vector<int> bgr = { 0, 0, 0 };
		design.rgb(hue, 255 - 100, 255, bgr);

		for (int i = 0; i < catmullLine.size(); i++){
			for (int j = 0; j < catmullLine[i].size(); j++){
				int y = catmullLine[i].at(j).first;
				int x = catmullLine[i].at(j).second;
				circle(srcImg, cv::Point(x, y), 0.5, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
			}
		}
	}
	void drawLine(cv::Mat &srcImg, vector<pair<int, int>> &contours, int hue){
		NeonDesign design;
		vector<int> bgr = { 0, 0, 0 };
		vector<pair<int, int>> ctr;
		design.rgb(hue, 255, 255 - 100, bgr);
		for (int i = 0; i < contours.size(); i++){
			int y = contours.at(i).first;
			int x = contours.at(i).second;
			if (i >= contours.size() || i + 1 >= contours.size() || i + 2 >= contours.size() || i + 3 >= contours.size()) break;
			if (i == 0){
				for (double t = 0; t <= 1.0; t += 0.1){
					y = catmullRomFirstLast(contours.at(0).first, contours.at(1).first, t);
					x = catmullRomFirstLast(contours.at(0).second, contours.at(1).second, t);
					ctr.push_back(make_pair(y, x));
					circle(srcImg, cv::Point(x, y), 2, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
				}
			}
			for (double t = 0; t <= 1.0; t += 0.05){
				y = catmullRom(contours.at(i).first, contours.at(i + 1).first, contours.at(i + 2).first, contours.at(i + 3).first, t);
				x = catmullRom(contours.at(i).second, contours.at(i + 1).second, contours.at(i + 2).second, contours.at(i + 3).second, t);
				ctr.push_back(make_pair(y, x));
				circle(srcImg, cv::Point(x, y), 2, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
			}
			if (i == contours.size() - 4){
				for (double t = 0; t <= 1.0; t += 0.1){
					y = catmullRomFirstLast(contours.at(contours.size() - 2).first, contours.at(contours.size() - 1).first, t);
					x = catmullRomFirstLast(contours.at(contours.size() - 2).second, contours.at(contours.size() - 1).second, t);
					ctr.push_back(make_pair(y, x));
					circle(srcImg, cv::Point(x, y), 2, cv::Scalar(bgr.at(0), bgr.at(1), bgr.at(2)), -1, 4);
				}
			}
		}	
		catmullLine.push_back(ctr);

	}
};