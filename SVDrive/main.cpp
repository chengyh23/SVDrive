/* 在data_root/calib_rectify中新增的文件：
intrinsics.yml
extrinsics.yml
img_0_rectified.jpg
img_1_rectified.jpg
rectified_canvas.jpg */
#define _CRT_SECURE_NO_WARNINGS 1
#include"stereo_calib.h"
#include"stereo_rect.h"
#include"disparity_filter.h"
#include"sad.h"
#include<stdbool.h>
#include<ctime>

string output_path;
const double baseline=1.0;

string extConvert(string src_full, const string dst_ext) {
	return src_full.replace(src_full.end() - 3, src_full.end(), dst_ext);
}

void on_mouse(int event, int y, int x, int flags, void* ustc)
{
	//std::cout << event << " "<< cv::EVENT_LBUTTONDOWN << " " << flags << std::endl;      
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		cv::FileStorage fs(output_path+"intrinsics.yml", cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", (output_path + "intrinsics.yml").c_str());
			exit(-1);
		}
		ofstream log(output_path + "log_click", ofstream::app);
		cv::Mat M1;
		fs["M1"] >> M1;
			
		double cx = M1.ptr<double>(0)[2];
		double fx = M1.ptr<double>(0)[0];
		double cy = M1.ptr<double>(1)[2];
		double fy = M1.ptr<double>(1)[1];
		double doffs = 0.0;
		std::cout << "(" << x << ", " << y << ")" << std::endl;
		log << "(" << x << ", " << y << ")" <<'\t';
		int d = (int)((cv::Mat*)ustc)->ptr<uchar>(x)[y];
		double dz = fx * baseline / ((double)d + doffs); // Zc = baseline * f / (d + doffs) 
		double dx = (cx - (double)y) * dz / fx; // Xc向右，Yc向下为正 
		double dy = (cy - (double)x) * dz / fy;
		//double dis = 6128.1*pow(d,-1.221);
		double D = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
		double dis = (D - 6.9092) / 0.7158;//补偿公式
		std::cout << std::setprecision(4) << dx << "," << dy << "," << dz << "," << D << std::endl;
		std::cout << std::setprecision(4) << d << ", " << dis << std::endl;
		log << std::setprecision(4) << dx << "," << dy << "," << dz << "," << D << '\t';
		log << std::setprecision(4) << d << ", " << dis << std::endl;
		log.close();
	}
}

int main(int argc, char** argv)
{
	string keys =
		"{help ||print this message }"
		"{data_root|E:\\Works\\DaChuang\\data\\20191211\\|}"
		"{do_calib|0|}"
		"{do_rect |1|}"

		"{w|7|boardSize.width}"
		"{h|4|boardSize.height}"
		"{s|1.0|squareSize}"
		
		"{l              |20191211_080322_049__0_22.80.jpg| left image(relative path) }"
		"{r              |20191211_080322_049__1_22.80.jpg| right image(relative path) }"

		"{algorithm      |bm              | stereo matching method (bm or sgbm )                              }"
		// these parameters are not used unless you use sgbm/bm---------------------
		"{filter         |fbs_conf          | used post-filtering (wls_conf or wls_no_conf or fbs_conf)         }"
		"{no-downscale   |                  | force stereo matching on full-sized views to improve quality      }"
		"{dst_conf_path  |None              | optional path to save the confidence map used in filtering        }"
		"{vis_mult       |1.0               | coefficient used to scale disparity map visualizations            }"
		"{max_disparity  |256               | parameter of stereo matching                                      }"
		"{window_size    |-1                | parameter of stereo matching                                      }"
		"{wls_lambda     |8000.0            | parameter of post-filtering                                       }"
		"{wls_sigma      |1.5               | parameter of post-filtering                                       }"
		// --------------------------------------------------------------------------
		;

	cv::CommandLineParser parser(argc, argv, keys);	
	if (parser.has("help")) parser.printMessage();
	string data_root = parser.get<string>("data_root");
	int do_calib = parser.get<int>("do_calib");
	int do_rect = parser.get<int>("do_rect");

	Size boardSize;
	boardSize.width = parser.get<int>("w");
	boardSize.height = parser.get<int>("h");
	float squareSize = parser.get<float>("s");
	cv::String lname = data_root+parser.get<cv::String>("l");
	cv::String rname = data_root+parser.get<cv::String>("r");
	
	cv::String dst_conf_path = parser.get<cv::String>("dst_conf_path");
	// argument: method of match stereopair
	cv::String algorithm = parser.get<cv::String>("algorithm");
	

	if (!parser.check())
	{
		parser.printErrors();
		return 1;
	}

	string imagelistfn = samples::findFile(data_root + "img_list.xml");
	vector<string> imagelist;
	bool ok = readStringList(imagelistfn, imagelist, data_root);
	if (!ok || imagelist.empty())
		cout << "can not open " << imagelistfn << " or the string list is empty" << endl;

	output_path = data_root + "output\\";
	string disp_path = output_path + "disp.jpg";
	string disp_color_path = output_path + "disp_color.jpg";
	if(do_calib)
		StereoCalib(imagelist,output_path , boardSize, squareSize, false, true);
	if (!do_rect) return 0;
	cv::Mat left = cv::imread(lname, IMREAD_COLOR);
	cv::Mat right = cv::imread(rname,IMREAD_COLOR);
	if (left.empty() || right.empty()) {
		cout << "can not read left or right image" << endl;
		return -1;
	}
	std::cout << left.size() << std::endl;
	StereoRect(output_path + "intrinsics.yml", output_path + "extrinsics.yml", left ,right);
	cv::imshow("lr", left);
	cv::imshow("rr", right);
	cv::imwrite(output_path +"rect_"+ rname.substr(rname.rfind('\\') + 1), right);
	cv::imwrite(output_path +"rect_"+ lname.substr(lname.rfind('\\') + 1), left);

	Mat disparity;
	if (algorithm == "sad") {
		SAD sad(5, 10);
		disparity = sad.computeSAD(left, right);
		cv::imwrite(disp_path, disparity);
	}
	else if (algorithm == "sgbm"|| algorithm == "bm") {
		// argument: method of disparity_filter
		cv::String filter = parser.get<cv::String>("filter");
		// agrument: force stereo matching on full-sized views to improve quality
		bool no_downscale = parser.has("no-downscale");

		int max_disp = parser.get<int>("max_disparity");
		int wsize = parser.get<int>("window_size");
		if (parser.get<int>("window_size") >= 0) //user provided window_size value
			wsize = parser.get<int>("window_size");
		else {
			if (algorithm == "sgbm")
				wsize = 3; //default window size for SGBM
			else if (!no_downscale && algorithm == "bm" && filter == "wls_conf")
				wsize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
			else
				wsize = 15; //default window size for BM on full-sized views
		}
		double lambda = parser.get<double>("wls_lambda");
		double sigma = parser.get<double>("wls_sigma");
		double vis_mult = parser.get<double>("vis_mult");
		cout << max_disp << std::endl;

		disparity_filter(disparity, left, right, algorithm, filter, no_downscale, wsize, max_disp, lambda, sigma, vis_mult);
		cv::imwrite(disp_path, disparity);
	}
	else if (algorithm == "gc") {
		/*
		输入：png图像
		cmd调用编译好的graph_cut.exe
		输出：超参数dmin和dmax, disp.tif, disp.png
		*/

		lname = extConvert(lname, "png");
		rname = extConvert(rname, "png");
		imwrite(lname, left);
		imwrite(rname, right);
		string command = "graph_cut " + lname + ' ' + rname
			+ " -15 0 " + extConvert(disp_path, ".tif") + " -o " + extConvert(disp_path, ".png");
		system(command.c_str());
		disparity = imread(extConvert(disp_path, ".png"), 0);
		command = "rm " + lname; system(command.c_str());
		command = "rm " + rname; system(command.c_str());
	}
	ofstream log(output_path + "log_click", ofstream::app);
	time_t now = time(0);
	tm* ltm = localtime(&now);
	log << 1990 + ltm->tm_year << '/' << ltm->tm_mon << '/' << ltm->tm_mday << ' ' << ltm->tm_hour << ':' << ltm->tm_min << ' ' << lname << endl;
	log.close();

	Mat temp_disp, color_disp;
	disparity.convertTo(temp_disp, CV_8UC1);
	cv::applyColorMap(temp_disp, color_disp, cv::COLORMAP_JET);
	imwrite(disp_color_path, color_disp);
	cv::setMouseCallback("lr", on_mouse, &disparity); cv::waitKey();
	return 0;
}


