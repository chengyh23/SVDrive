**Quick Start**

- customize the *data_root* in which contains images pairs and imglist(an xml file)
- set *do_calib* to 1 if you haven't calibrated the camera using images in your data folder
- customize the *w, h, s* (parameter of the chessboard you use) 
- customize *l* and *r* so that this pair of images are used for rectifying & computing disparity
- set *algorithm* you want to use for stereo matching
- if you are not using sgbm/bm for stereo matching, you don't have to care about these parameter: *filter. no-downscale, max_disparity, window_size, wls_lambda, wls_sigma, vis_mult* 
- Run main.cpp. (StereoCalib->StereoRect->compute disparity)

**Core Function, Tools and Utils**

|Function  |introduction			|
|:-------- |:--				|
|StereoCalib|use findChessboardCorners, initCameraMatrix2D, stereoCalibrate in opencv library
|StereoRect|use stereoRectify, initUndistortRectifyMap, remap in opencv library
|computeSAD|stereo matching, SAD|
|disparity_filter|stereo matching, SGBM/BM|
|extConvert|manipulate string of image's file name. Example: input (img.jpg,png), output img.png|
|on_mouse  |define the action when the user clicks some point of the disparity image|
 

**Folder Hierarchy**

/data/20191211/
	
&emsp;|- imglist.xml

&emsp;|- 20191211_080322_049__0\_22.80.jpg

&emsp;|- 20191211_080322_049__1\_22.80.jpg

&emsp;|- ......

&emsp;|- output/



&emsp;&emsp;|- extrinsics.yml

&emsp;&emsp;|- intrinsics.yml

&emsp;&emsp;|- rectified\_20191211_080322_049__0\_22.80.jpg

&emsp;&emsp;|- rectified\_20191211_080322_049__1\_22.80.jpg

&emsp;&emsp;|- disp.jpg

&emsp;&emsp;|- disp_color.jpg

&emsp;&emsp;|- log_click

**Project Properties**

in Project Properties Manager, you can add existing property sheet(SVDrive/env_opencv.props) when you are creating new projects.
