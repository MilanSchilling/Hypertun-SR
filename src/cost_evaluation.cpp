#include "cost_evaluation.hpp"
#include <iostream>

// cost_evaluation:
// inputs: 
// - I_l : image left
// - I_r : image right
// - D_it: Interpolated disparity
//
// outputs:
// - C_it: Cost associated to D_it 
cv::Mat cost_evaluation(cv::Mat I_l, cv::Mat I_r, cv::Mat D_it){
	// TODO: census container to bool container!
	std::cout << "cost_evaluation.cpp" << std::endl;

	// define return 
	cv::Mat C_it = cv::Mat::zeros(D_it.rows, D_it.cols, CV_16U);
	//cv::Mat C_it = D_it;

	// Get image height and width
	int H = I_l.rows;
	int W = I_r.cols;

	// pad the images
	int border = 2;
	cv::Mat I_r_p = cv::Mat(I_r.rows + border*2, I_r.cols + border*2, I_r.depth());
	cv::Mat I_l_p = cv::Mat(I_l.rows + border*2, I_l.cols + border*2, I_l.depth());

	cv::copyMakeBorder(I_r, I_r_p, border, border, border, border, cv::BORDER_REPLICATE);
	cv::copyMakeBorder(I_l, I_l_p, border, border, border, border, cv::BORDER_REPLICATE);

	// loop over interpolated disparities
	for (int i = 0; i < H; ++i){
		for (int j = 0; j < W; j++){
			std::cout << "test0" << std::endl;
			int disp = D_it.at<int>(i,j); // TODO: verify order of i,j!
			std::cout << "test1" << std::endl;
			//evaluate cost for given disparity
			int i_pad = i + border;
			int j_pad = j + border;
			std::cout << "test2" << std::endl;
			cv::Mat census_l = cv::Mat::zeros(1, 24, CV_8U);
			cv::Mat census_r = cv::Mat::zeros(1, 24, CV_8U);
			std::cout << "test3" << std::endl;
			// get census of left image
			census_l = census(I_l_p, i_pad, j_pad);
			std::cout << "test4" << std::endl;
			// get census of right image at the interpolated disparity
			census_r = census(I_r_p, i_pad, j_pad + disp);
			std::cout << "test5" << std::endl;
			// calculate cost
			ushort cost = 0;
			for (int it = 0; it < 25; ++it){
				if (census_l.at<int>(0,it) == census_r.at<int>(0,it))
					cost = cost + 1;
			}
			std::cout << "test6" << std::endl;
			std::cout << D_it.type() << std::endl;
			// write cost to C_it
			std::cout << "i/j = " << i << "/" << j << std::endl;
			std::cout << "cost = " << cost << std::endl;
			C_it.at<int>(i,j) = cost;
			std::cout << "C_it at i/j = " << C_it.at<ushort>(i,j) << std::endl;
			std::cout << "test7" << std::endl;
		}
	}
	

	
	return C_it;
}



cv::Mat census (cv::Mat paddedImg, int i_pad, int j_pad){
	// container for census values
	cv::Mat census = cv::Mat::zeros(1, 24, CV_8U);

	// get intensity of center pixel
	uchar center = paddedImg.at<uchar>(i_pad, j_pad);

	int count = 0;

	// loop over window
	for (int a = -2; a < 3; ++a){
		for (int b = -2; b < 3; ++b){
			// leave the center pixel out
			if  (!((a == 0) && (b == 0))){
				// get the value of the neighbor pixel
				uchar px = paddedImg.at<uchar>(i_pad + a, j_pad + b);
				// set census to 1 if intenity is less
				if (px < center){
					census.at<int>(0,count) = 1;
					count++;
				} else{
					// else leave at zero
					count++;
				}
			}
		}
	}
	return census;
}