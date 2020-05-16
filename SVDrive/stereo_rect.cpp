#include"stereo_rect.h"

void StereoRect(string intrinsic_filename,string extrinsic_filename,cv::Mat& left,cv::Mat& right) {
	cv::Size img_size = left.size();
	cv::FileStorage fs(intrinsic_filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", intrinsic_filename.c_str());
		exit(-1);
	}

	cv::Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	fs.open(extrinsic_filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filename.c_str());
		exit(-1);
	}

	cv::Mat R, T, R1, P1, R2, P2, Q;
	fs["R"] >> R;
	fs["T"] >> T;

	cv::Rect roi1, roi2;
	cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
	fs.open(extrinsic_filename, cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
	{
		printf("Failed to open file %s\n", extrinsic_filename.c_str());
		exit(-1);
	}
	

	cv::Mat map11, map12, map21, map22;
	cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	cv::Mat img1r, img2r;
	cv::remap(left, left, map11, map12, cv::INTER_LINEAR);
	cv::remap(right, right, map21, map22, cv::INTER_LINEAR);

	/*cv::Rect roi = roi1 & roi2;
	left = img1r(roi);
	right = img2r(roi);*/
	
}


