#pragma once
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <Windows.h>
#include <time.h>

using namespace std;

class NeonDesign{
private:
public:
	NeonDesign(){}
	~NeonDesign(){}
	void rgb(double H, double S, double V, vector<int> &bgr){
		int Hi;
		double f, p, q, t;

		Hi = ((int)(H / 60)) % 6;
		f = H / 60 - (int)(H / 60);
		p = V * (1 - (S / 255));
		q = V * (1 - f * (S / 255));
		t = V * (1 - (1 - f) * (S / 255));

		switch (Hi) {
		case 0: bgr.at(0) = p; bgr.at(1) = t; bgr.at(2) = V; break;
		case 1: bgr.at(0) = p; bgr.at(1) = V; bgr.at(2) = q; break;
		case 2: bgr.at(0) = t; bgr.at(1) = V; bgr.at(2) = p; break;
		case 3: bgr.at(0) = V; bgr.at(1) = q; bgr.at(2) = p; break;
		case 4: bgr.at(0) = V; bgr.at(1) = p; bgr.at(2) = t; break;
		case 5: bgr.at(0) = q; bgr.at(1) = p; bgr.at(2) = V; break;
		}
	}
};