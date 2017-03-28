#include "disparity_refinement.hpp"
#include <iostream>

// disparity refinement:
// inputs:
// -D_it : Interpolated disparity
// -C_it : Normalized cost associated to D_it
//
// outputs:
// -updated D_f : final disparity map
// -updated C_f : final cost map
// C_g : Cost associated with regions of high confidence matches
// C_b : Cost associated with regions of invalid disparities
//######################################################################
// This function loops through the cost map and:
// - stores disparities and costs into final matrices if better than the finals
// - stores the highest cost per occupancy window into C_b, along with 
// 	 the associated pixel coordinates
// - stores the lowest cost per occupancy window into C_g, along with 
//   the associated pixel coordinates
void disparity_refinement(cv::Mat &D_it, cv::Mat &C_it, 
							cv::Mat &D_f, cv::Mat &C_f,
							cv::Mat &C_g, cv::Mat &C_b){
	// TODO: pass parameter struct to this function
	std::cout << "disparity_refinement.cpp" << std::endl;
	int sz_occ = 32;
	double t_hi = 0.8;
	double t_lo = 0.3;

	// get image dimensions
	const int H = D_it.rows;
	const int W = D_it.cols;

	// loop over C_it
	for (int i = 0; i < H; ++i){
		for (int j = 0; j < W; ++j){
			// Establish occupancy grid for resampled points
			int i_bar = int(i / sz_occ);
			int j_bar = int(j / sz_occ); 

			// If matching cost is lower than previous best final cost
			if (C_it.at<double>(i,j) < C_f.at<double>(i,j)){
				// store current disparity and cost to final
				D_f.at<double>(i,j) = D_it.at<double>(i,j);
				C_f.at<double>(i,j) = C_it.at<double>(i,j);
			}

			// If matching cost is lower than previous best valid cost
			if (C_it.at<double>(i,j) < t_lo && C_it.at<double>(i,j) < C_g.at<double>(i_bar,j_bar,3)){
				C_g.at<double>(i_bar,j_bar, 0) = i;
				C_g.at<double>(i_bar,j_bar, 1) = j;
				C_g.at<double>(i_bar,j_bar, 2) = D_it.at<double>(i,j);
				C_g.at<double>(i_bar,j_bar, 3) = C_it.at<double>(i,j);
			}

			// If matching cost is higher than previous worst invalid cost
			if (C_it.at<double>(i,j) > t_hi && C_it.at<double>(i,j) > C_b.at<double>(i_bar, j_bar, 2)){
				C_b.at<double>(i_bar,j_bar, 0) = i;
				C_b.at<double>(i_bar,j_bar, 1) = j;
				C_b.at<double>(i_bar,j_bar, 2) = C_it.at<double>(i,j);
			}
		}
	}
}