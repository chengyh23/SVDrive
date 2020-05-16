#include"disparity_filter.h"

cv::Rect computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance)
{
	int min_disparity = matcher_instance->getMinDisparity();
	int num_disparities = matcher_instance->getNumDisparities();
	int block_size = matcher_instance->getBlockSize();

	int bs2 = block_size / 2;
	int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

	int xmin = maxD + bs2;
	int xmax = src_sz.width + minD - bs2;
	int ymin = bs2;
	int ymax = src_sz.height - bs2;

	cv::Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
	return r;
}

void disparity_filter(cv::Mat& disparity, const cv::Mat _left, const cv::Mat _right, const cv::String algo, const cv::String filter, const bool no_downscale,
	const int wsize, int max_disp, const double lambda, const double sigma, const double vis_mult)
{
	cv::Mat left = _left;
	cv::Mat right = _right;

	cv::Mat left_for_matcher, right_for_matcher, guide;
	cv::Mat left_disp, right_disp;
	cv::Mat filtered_disp, filtered_disp_vis, solved_disp;
	cv::Mat conf_map = cv::Mat(left.rows, left.cols, CV_8U);
	conf_map = cv::Scalar(255);
	cv::Rect ROI;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
	double matching_time, filtering_time;
	double solving_time = 0;
	if (max_disp <= 0 || max_disp % 16 != 0) {
		std::cerr << "Incorrect max_disparity value: it should be positive and divisible by 16";
		exit(EXIT_FAILURE);
	}
	if (wsize <= 0 || wsize % 2 != 1) {
		std::cerr << "Incorrect window_size value: it should be positive and odd";
		exit(EXIT_FAILURE);
	}

	if (filter == "wls_conf") { // filtering with confidence (significantly better quality than wls_no_conf)
		if (!no_downscale) {
			// downscale the views to speed-up the matching stage, as we will need to compute both left
			// and right disparity maps for confidence map computation
			//! [downscale]
			max_disp /= 2;
			if (max_disp % 16 != 0)
				max_disp += 16 - (max_disp % 16);
			cv::resize(left, left_for_matcher, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
			cv::resize(right, right_for_matcher, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR_EXACT);
			//! [downscale]
		}
		else {
			left_for_matcher = left.clone();
			right_for_matcher = right.clone();
		}

		if (algo == "bm") {
			//! [matching]
			cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(max_disp, wsize);
			wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
			cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

			cv::cvtColor(left_for_matcher, left_for_matcher, cv::COLOR_BGR2GRAY);
			cv::cvtColor(right_for_matcher, right_for_matcher, cv::COLOR_BGR2GRAY);

			matching_time = (double)cv::getTickCount();
			left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
			matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();
			//! [matching]
		}
		else if (algo == "sgbm") {
			cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, max_disp, wsize);
			left_matcher->setP1(24 * wsize * wsize);
			left_matcher->setP2(96 * wsize * wsize);
			left_matcher->setPreFilterCap(63);
			left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
			wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
			cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

			matching_time = (double)cv::getTickCount();
			left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
			matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();
		}
		else {
			std::cerr << "Unsupported algorithm";
			exit(EXIT_FAILURE);
		}

		//! [filtering]
		wls_filter->setLambda(lambda);
		wls_filter->setSigmaColor(sigma);
		filtering_time = (double)cv::getTickCount();
		wls_filter->filter(left_disp, left, filtered_disp, right_disp);
		filtering_time = ((double)cv::getTickCount() - filtering_time) / cv::getTickFrequency();
		//! [filtering]
		conf_map = wls_filter->getConfidenceMap();

		// Get the ROI that was used in the last filter call:
		ROI = wls_filter->getROI();
		if (!no_downscale) {
			// upscale raw disparity and ROI back for a proper comparison:
			cv::resize(left_disp, left_disp, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
			left_disp = left_disp * 2.0;
			ROI = cv::Rect(ROI.x * 2, ROI.y * 2, ROI.width * 2, ROI.height * 2);
		}
	}
	else if (filter == "fbs_conf") { // filtering with confidence (significantly better quality than wls_no_conf)
		if (!no_downscale) {
			// downscale the views to speed-up the matching stage, as we will need to compute both left
			// and right disparity maps for confidence map computation
			//! [downscale_wls]
			max_disp /= 2;
			if (max_disp % 16 != 0)
				max_disp += 16 - (max_disp % 16);
			cv::resize(left, left_for_matcher, cv::Size(), 0.5, 0.5);
			cv::resize(right, right_for_matcher, cv::Size(), 0.5, 0.5);
			//! [downscale_wls]
		}
		else {
			left_for_matcher = left.clone();
			right_for_matcher = right.clone();
		}
		guide = left_for_matcher.clone();

		if (algo == "bm") {
			//! [matching_wls]
			cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(max_disp, wsize);
			wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
			cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

			cv::cvtColor(left_for_matcher, left_for_matcher, cv::COLOR_BGR2GRAY);
			cv::cvtColor(right_for_matcher, right_for_matcher, cv::COLOR_BGR2GRAY);

			matching_time = (double)cv::getTickCount();
			left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
			matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();
			//! [matching_wls]
		}
		else if (algo == "sgbm") {
			cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, max_disp, wsize);
			left_matcher->setP1(24 * wsize * wsize);
			left_matcher->setP2(96 * wsize * wsize);
			left_matcher->setPreFilterCap(63);
			left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
			wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
			cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

			matching_time = (double)cv::getTickCount();
			left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
			matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();
		}
		else {
			std::cerr << "Unsupported algorithm";
			exit(EXIT_FAILURE);
		}

		//! [filtering_wls]
		wls_filter->setLambda(lambda);
		wls_filter->setSigmaColor(sigma);
		filtering_time = (double)cv::getTickCount();
		wls_filter->filter(left_disp, left, filtered_disp, right_disp, ROI, right);
		filtering_time = ((double)cv::getTickCount() - filtering_time) / cv::getTickFrequency();
		//! [filtering_wls]

		conf_map = wls_filter->getConfidenceMap();

		cv::Mat left_disp_resized;
		cv::resize(left_disp, left_disp_resized, left.size());

		// Get the ROI that was used in the last filter call:
		ROI = wls_filter->getROI();
		//std::cout << ROI << std::endl;
		if (!no_downscale) {
			// upscale raw disparity and ROI back for a proper comparison:
			cv::resize(left_disp, left_disp, cv::Size(), 2.0, 2.0);
			left_disp = left_disp * 2.0;
			ROI = cv::Rect(ROI.x * 2, ROI.y * 2, ROI.width * 2, ROI.height * 2);
		}

#ifdef HAVE_EIGEN
		//! [filtering_fbs]
		solving_time = (double)cv::getTickCount();
		// wls_filter->filter(left_disp,left,filtered_disp,right_disp);
		// fastBilateralSolverFilter(left, filtered_disp, conf_map, solved_disp, 16.0, 16.0, 16.0);
		fastBilateralSolverFilter(left, left_disp_resized, conf_map, solved_disp, 16.0, 16.0, 16.0);
		solving_time = ((double)cv::getTickCount() - solving_time) / cv::getTickFrequency();
		solved_disp.convertTo(solved_disp, CV_8UC1);
		cv::equalizeHist(solved_disp, solved_disp);
		//! [filtering_fbs]
#endif
	}
	else if (filter == "wls_no_conf") {
		/* There is no convenience function for the case of filtering with no confidence, so we
		will need to set the ROI and matcher parameters manually */

		left_for_matcher = left.clone();
		right_for_matcher = right.clone();

		if (algo == "bm") {
			cv::Ptr<cv::StereoBM> matcher = cv::StereoBM::create(max_disp, wsize);
			matcher->setTextureThreshold(0);
			matcher->setUniquenessRatio(0);
			cv::cvtColor(left_for_matcher, left_for_matcher, cv::COLOR_BGR2GRAY);
			cv::cvtColor(right_for_matcher, right_for_matcher, cv::COLOR_BGR2GRAY);
			ROI = computeROI(left_for_matcher.size(), matcher);
			wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
			wls_filter->setDepthDiscontinuityRadius((int)ceil(0.33 * wsize));

			matching_time = (double)cv::getTickCount();
			matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();
		}
		else if (algo == "sgbm") {
			cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(0, max_disp, wsize);
			matcher->setUniquenessRatio(0);
			matcher->setDisp12MaxDiff(1000000);
			matcher->setSpeckleWindowSize(0);
			matcher->setP1(24 * wsize * wsize);
			matcher->setP2(96 * wsize * wsize);
			matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
			ROI = computeROI(left_for_matcher.size(), matcher);
			wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
			wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5 * wsize));

			matching_time = (double)cv::getTickCount();
			matcher->compute(left_for_matcher, right_for_matcher, left_disp);
			matching_time = ((double)cv::getTickCount() - matching_time) / cv::getTickFrequency();
		}
		else {
			std::cerr << "Unsupported algorithm";
			exit(EXIT_FAILURE);
		}

		wls_filter->setLambda(lambda);
		wls_filter->setSigmaColor(sigma);
		filtering_time = (double)cv::getTickCount();
		wls_filter->filter(left_disp, left, filtered_disp, cv::Mat(), ROI);
		filtering_time = ((double)cv::getTickCount() - filtering_time) / cv::getTickFrequency();
	}
	else {
		std::cerr << "Unsupported filter";
		exit(EXIT_FAILURE);
	}

	cv::ximgproc::getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);

	//collect and print all the stats:
	std::cout.precision(2);
	std::cout << "Matching time:  " << matching_time << "s" << std::endl;
	std::cout << "Filtering time: " << filtering_time << "s" << std::endl;
	std::cout << "solving time: " << solving_time << "s" << std::endl;
	std::cout << std::endl;

	disparity = filtered_disp_vis;
}
