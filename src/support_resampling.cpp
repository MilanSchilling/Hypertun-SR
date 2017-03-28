#include "support_resampling.hpp"
#include <iostream>

void support_resampling(cv::Mat &C_g, cv::Mat &C_b, 
                            cv::Mat &S_it, cv::Mat &S_it_next){

	std::cout << "support_resampling.cpp" << std::endl;
	// TODO: pass params as argument
	int sz_occ = 32;

	S_it_next = S_it;

	// define vector of 2d points
	std::vector<cv::Point2d> X;


}