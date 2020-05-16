#pragma once
#include"opencv2/calib3d.hpp"
#include"opencv2/opencv.hpp"
#include<iostream>
using namespace std;
void StereoRect(string intrinsic_filename, string extrinsic_filename, cv::Mat& left,cv::Mat& right);