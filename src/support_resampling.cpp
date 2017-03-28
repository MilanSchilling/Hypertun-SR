#include "support_resampling.hpp"
#include <iostream>

void support_resampling(cv::Mat &C_g, cv::Mat &C_b, 
                            cv::Mat &S_it, parameters &param){

	std::cout << "support_resampling.cpp" << std::endl;

	// define vector of 2d points
	std::vector<cv::Point2d> X;

	const int H_bar = int(param.H / param.sz_occ);
	const int W_bar = int(param.W / param.sz_occ); 
	for (int i_bar = 0; i_bar < H_bar; ++i_bar){
		for (int j_bar = 0; j_bar < W_bar; ++j_bar){
			if (C_b.at<double>(i_bar, j_bar, 2) > param.t_hi){
				// store (u,v) for bad point for resampling
				cv::Point2d pt(C_b.at<double>(i_bar, j_bar, 0), C_b.at<double>(i_bar, j_bar, 1));
				// add bad point to X
				X.push_back(pt);
			}

			if (C_g.at<double>(i_bar, j_bar, 3) < param.t_lo){
				// store (u,v,d) for valid points
				cv::Point3d pt(C_g.at<double>(i_bar, j_bar, 0),
								C_b.at<double>(i_bar, j_bar, 1),
								C_b.at<double>(i_bar, j_bar, 2));
				// add valid support point to S_it_next
				S_it.push_back(pt);
			}
		}
	}

	// Re-estimate disparities via epipolar search
	// S_matched <-- SPARSE_EPIPOLAR_STEREO(I_l , I_r , X)
	// S_it_next <-- {S_it_next, S_matched}



}