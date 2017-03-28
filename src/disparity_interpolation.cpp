#include "disparity_interpolation.hpp"
#include <iostream>

void disparity_interpolation(cv::Mat &G, cv::Mat &T, cv::Mat &D_it){
	
	std::cout << "disparity_interpolation.cpp" << std::endl;

	int H = G.rows;
	int W = G.cols;
	cv::Mat D2 = cv::Mat(H, W, CV_32F, 0.0);

	for (int i = 0; i < H; ++i) {
		for (int j = 0; j < W; ++j) {
			int idx_trig = G.at<int>(i,j);
			//std::cout << idx_trig << std::endl;
			if (idx_trig >= 0) {
				//std::cout << idx_trig << std::endl;
				float a = T.at<float>(0,idx_trig);
				float b = T.at<float>(1,idx_trig);
				float c = T.at<float>(2,idx_trig);
				float d = T.at<float>(3,idx_trig);
				//std::cout << (d - a*i - b*j) / c << std::endl;
				D_it.at<float>(i,j) = (d - a*j - b*i) / c;
				D2.at<float>(i,j) = (d - a*j - b*i) / c;
			}
		}
	}

	//std::cout << D_it.at<float>(1,1) << std::endl;
	cv::Mat dst;
	D2.convertTo(dst, CV_8U, 0.5);
	//cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);

	cv::imshow("Interpolated Disparity", dst);
	cv::waitKey(0);
	
}