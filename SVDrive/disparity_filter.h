#pragma once
#include"opencv2/opencv.hpp"
#include"opencv2/ximgproc.hpp"
#include<iostream>
using namespace std;

cv::Rect computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance);
void disparity_filter(cv::Mat& disparity, const cv::Mat _left, const cv::Mat _right, const cv::String algo, const cv::String filter, const bool no_downscale,
	const int wsize, int max_disp, const double lambda, const double sigma, const double vis_mult);