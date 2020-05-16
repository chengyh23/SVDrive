#pragma once
/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warranty, support or any guarantee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
	 by Gary Bradski and Adrian Kaehler
	 Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
	 http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
	 Or: http://oreilly.com/catalog/9780596516130/
	 ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OPENCV WEBSITES:
	 Homepage:      http://opencv.org
	 Online docs:   http://docs.opencv.org
	 Q&A forum:     http://answers.opencv.org
	 GitHub:        https://github.com/opencv/opencv/
   ************************************************** */
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;


void StereoCalib(const vector<string>& imagelist, const string root_path,
	Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated = true);
bool readStringList(const string& filename, vector<string>& l, const string root_path);