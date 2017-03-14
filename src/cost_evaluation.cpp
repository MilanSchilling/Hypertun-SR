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

	std::cout << "cost_evaluation.cpp" << std::endl;

	// define return for debug use 
	cv::Mat C_it = D_it;
	return C_it;
}