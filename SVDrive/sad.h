#pragma once
#include<opencv2/core/core.hpp>
#include<iostream>
#include<iomanip>
using namespace cv;

class SAD {
public:
	SAD() :winSize(7), DSR(30) {}
	SAD(int winSize_, int DSR_) :winSize(winSize_), DSR(DSR_) {}
	Mat computeSAD(cv::Mat& L, cv::Mat& R);
private:
	int winSize;
	int DSR;
};

cv::Mat SAD::computeSAD(cv::Mat& L, cv::Mat& R) {
	int height = L.rows;
	int width = L.cols;
	cv::Mat kernel_L(cv::Size(winSize, winSize), CV_8U, cv::Scalar::all(0));
	cv::Mat kernel_R(cv::Size(winSize, winSize), CV_8U, cv::Scalar::all(0));
	cv::Mat disparity(height, width, CV_8U, cv::Scalar(0));
	std::cout << "running SAD %";
	for (int i = 0; i < width - winSize; i++) {
		for (int j = 0; j < height - winSize; j++) {
			kernel_L = L(cv::Rect(i, j, winSize, winSize));
			cv::Mat MM(1, DSR, CV_32F, cv::Scalar(0));
			for (int k = 0; k < DSR; k++) {
				int x = i - k;
				if (x >= 0) {
					kernel_R = R(cv::Rect(x, j, winSize, winSize));
					cv::Mat dif;
					cv::absdiff(kernel_L, kernel_R, dif);
					cv::Scalar Add = cv::sum(dif);
					float a = Add[0];
					MM.at<float>(k) = a;
				}
			}
			cv::Point minLoc;
			cv::minMaxLoc(MM, NULL, NULL, &minLoc, NULL);
			int loc = minLoc.x;
			disparity.at<char>(j, i) = loc * 16;
		}
		
		if (i % 100 == 0) {
			if (i) std::cout << "\b\b\b\b\b";
			std::cout << std::setprecision(2) << std::setw(5) << 100*double(i) / width;
		} 
	}
	std::cout << std::endl;
	return disparity;
}