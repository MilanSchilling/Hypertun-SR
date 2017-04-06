#include "support_resampling.hpp"
#include <iostream>

void support_resampling(cv::Mat &C_g, cv::Mat &C_b, 
                            cv::Mat &S_it, parameters &param){

	std::cout << "support_resampling.cpp" << std::endl;


	// define container for re-sampled or detected support pixels with unknown depths
	int sz_X[] = {1, 2}; 
	cv::Mat X (2, sz_X, CV_64F, cv::Scalar::all(0));

	// define vector of 2d points
	std::vector<cv::Point2d> X;

	// get grid size
	const int H_bar = int(param.H / param.sz_occ);
	const int W_bar = int(param.W / param.sz_occ); 

	for (int i_bar = 0; i_bar < H_bar; ++i_bar){
		for (int j_bar = 0; j_bar < W_bar; ++j_bar){
			if (C_b.at<double>(i_bar, j_bar, 2) > param.t_hi){
				// store (u,v) for bad point for resampling
				cv::Mat Pt = cv::Mat::zeros(1, 2, CV_64F);
				Pt.at<double>(0, 0) = C_b.at<double>(i_bar, j_bar, 0);
				Pt.at<double>(0, 1) = C_b.at<double>(i_bar, j_bar, 1);

				// add bad point to X
				X.push_back(Pt);
			}

			if (C_g.at<double>(i_bar, j_bar, 3) < param.t_lo){
				// store (u,v,d) for valid points
				cv::Mat Pt = cv::Mat::zeros(1, 3, CV_64F);
				Pt.at<double>(0, 0) = C_g.at<double>(i_bar, j_bar, 0);
				Pt.at<double>(0, 1) = C_g.at<double>(i_bar, j_bar, 1);
				Pt.at<double>(0, 2) = C_g.at<double>(i_bar, j_bar, 2);

				// add valid support point to S_it_next
				S_it.push_back(Pt);
			}
		}
	}

	// Re-estimate disparities via epipolar search
	// S_matched <-- SPARSE_EPIPOLAR_STEREO(I_l , I_r , X)
	// S_it_next <-- {S_it_next, S_matched}



}