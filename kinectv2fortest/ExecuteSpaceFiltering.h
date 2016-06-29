#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;


class ExecuteSpaceFiltering{
private:
public:
	vector<double> filter;
	double filtersize;
	cv::Mat image2;

	ExecuteSpaceFiltering(double filter_size){
		filtersize = filter_size;
	}
	~ExecuteSpaceFiltering(){}

	void createFilter(){
		double filter_value = 1 / filtersize;
		for (int i = 0; i < filtersize; i++)
			filter.push_back(filter_value);
	}
	void createNeighbour(int size, vector<pair<int, int>> &neighbour){
		// 範囲チェック
		if (size < 3) {
			size = 3;
		}
		if (size > 15) {
			size = 15;
		}
		size--;
		size /= 2;

		createFilter();

		for (int y = -size; y <= size; y++) {
			for (int x = -size; x <= size; x++) {
				neighbour.push_back(make_pair(y, x));
			}
		}
	}

	void createNeighbourGaussian(int size, vector<pair<int, int>> &neighbour){
		// 範囲チェック
		if (size < 3) {
			size = 3;
		}
		if (size > 15) {
			size = 15;
		}
		size--;
		size /= 2;

		float sigma = 1.0;
		float sum = 0;
		for (int y = -size; y <= size; y++) {
			for (int x = -size; x <= size; x++) {
				double gf = GaussianFunc(y, x, sigma);
				filter.push_back(gf);
				neighbour.push_back(make_pair(y, x));
				sum += gf;
			}
		}
		for (int i = 0; i < filter.size(); i++) {
			filter.at(i) /= sum;
		}

	}

	double GaussianFunc(int y, int x, float sigma) {
		float pi = (float)M_PI;
		float sigma2 = sigma * sigma;
		double gauss_const = 1.0 / (2.0 * pi * sigma * sigma);
		double f = gauss_const * exp(-(x * x + y * y) / (2.0 * sigma * sigma));
		return f;

	}

	void applyFiltering(int y, int x, vector<pair<int, int>> &neighbour, vector<double> &bgr, cv::Mat &srcImg){
		for (int i = 0; i < neighbour.size(); i++){
			int dy = y + neighbour.at(i).first;
			int dx = x + neighbour.at(i).second;
			if (dy < 0 || dy >= srcImg.rows || dx < 0 || dx >= srcImg.cols) continue;
			bgr.at(0) += srcImg.at<cv::Vec3b>(dy, dx)[0] * filter.at(i);
			bgr.at(1) += srcImg.at<cv::Vec3b>(dy, dx)[1] * filter.at(i);
			bgr.at(2) += srcImg.at<cv::Vec3b>(dy, dx)[2] * filter.at(i);
		}
	}

	void executeSpaceFilteringYX(int y, int x, cv::Mat &srcImg, cv::Mat &resultImg){
		vector<pair<int, int>> neighbour;
		vector<double> bgr(3, 0.0);
		int width = srcImg.cols;
		int height = srcImg.rows;
		filter.clear();
		createNeighbour(sqrt(filtersize), neighbour);
		applyFiltering(y, x, neighbour, bgr, srcImg);

		// valueR, valueG, valueB の値を0～255の範囲にする
		if (bgr.at(2) < 0.0) bgr.at(2) = 0.0;
		if (bgr.at(2) > 255.0) bgr.at(2) = 255.0;
		if (bgr.at(1) < 0.0) bgr.at(1) = 0.0;
		if (bgr.at(1) > 255.0) bgr.at(1) = 255.0;
		if (bgr.at(0) < 0.0) bgr.at(0) = 0.0;
		if (bgr.at(0) > 255.0) bgr.at(0) = 255.0;

		resultImg.at<cv::Vec3b>(y, x)[0] = bgr.at(0);
		resultImg.at<cv::Vec3b>(y, x)[1] = bgr.at(1);
		resultImg.at<cv::Vec3b>(y, x)[2] = bgr.at(2);
	}

	//
	// 空間フィルタリングを用いた画像処理の例
	//
	void executeSpaceFilteringAll(cv::Mat &srcImg) {
		vector<pair<int, int>> neighbour;
		vector<double> bgr(3, 0.0);
		image2 = cv::Mat(srcImg.size(), srcImg.type(), cvScalarAll(255));
		int width = srcImg.cols;
		int height = srcImg.rows;
		createNeighbour(sqrt(filtersize), neighbour);

		//
		// 各スキャンラインごとに
		//
		for (int i = 0; i < height; i++) {

			//
			// 各画素ごとに
			//
			for (int j = 0; j < width; j++) {
				bgr = { 0.0, 0.0, 0.0 };
				int y = i;
				int x = j;
				applyFiltering(y, x, neighbour, bgr, srcImg);

				// valueR, valueG, valueB の値を0～255の範囲にする
				if (bgr.at(2) < 0.0) bgr.at(2) = 0.0;
				if (bgr.at(2) > 255.0) bgr.at(2) = 255.0;
				if (bgr.at(1) < 0.0) bgr.at(1) = 0.0;
				if (bgr.at(1) > 255.0) bgr.at(1) = 255.0;
				if (bgr.at(0) < 0.0) bgr.at(0) = 0.0;
				if (bgr.at(0) > 255.0) bgr.at(0) = 255.0;

				image2.at<cv::Vec3b>(y, x)[0] = bgr.at(0);
				image2.at<cv::Vec3b>(y, x)[1] = bgr.at(1);
				image2.at<cv::Vec3b>(y, x)[2] = bgr.at(2);
			}
		}
	}
};