#include "disparity_refinement.hpp"
#include <iostream>

void disparity_refinement(cv::Mat &D_it, cv::Mat &C_it, cv::Mat &C_f){
	// TODO: pass parameter struct to this function
	std::cout << "disparity_refinement.cpp" << std::endl;
	int sz_occ = 32;
	double t_hi = 0.8;
	double t_lo = 0.3;

	// get image dimensions
	const int H = D_it.rows;
	const int W = D_it.cols;

	// divide image into discrete parts
	const int H_bar = int(D_it.rows / sz_occ);
	const int W_bar = int(D_it.cols / sz_occ);

	// creating container for good and bad matches
	
	int sz_g[] = {H_bar, W_bar, 4};
	int sz_b[] = {H_bar, W_bar, 3};
	cv::Mat C_g (3, sz_g, CV_64F, cv::Scalar::all(0));
	cv::Mat C_b (3, sz_b, CV_64F, cv::Scalar::all(0));

	// set thresholds
	// TODO: do this within inizialisation above!
	for (int i = 0; i < H_bar; ++i){
		for (int j = 0; j < W_bar; ++j){
			C_g.at<double>(i,j,3) = t_lo;
			C_b.at<double>(i,j,2) = t_hi;
		}
	}

	// loop over C_it
	for (int i = 0; i < H; ++i){
		for (int j = 0; j < W; ++j){
			// Establish occupancy grid for resampled points
			int i_bar = int(i / sz_occ);
			int j_bar = int(j / sz_occ); 

			// If matching cost is lower than previous best final cost
			if (C_it.at<double>(i,j) < C_f.at<double>(i,j)){
				// D_f(u,v) <-- D_it(u,v)
				// C_f(u,v) <-- C_it(u,v)
			}

			// If matching cost is lower than previous best valid cost
			if (C_it.at<double>(i,j) < t_lo && C_it.at<double>(i,j) < C_g.at<double>(i_bar,j_bar,3)){
				// C_g(u',v') <-- (u, v, D_it(u,v))
			}

			// If matching cost is higher than previous worst invalid cost
			if (C_it.at<double>(i,j) > t_hi && C_it.at<double>(i,j) > C_b.at<double>(i_bar, j_bar, 2)){
				// C_b(u',v') <-- (u, v, C_it(u,v))
			}
		}
	}
}