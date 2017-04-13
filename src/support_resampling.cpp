#include "support_resampling.hpp"
#include <iostream>

// support_resampling :
// inputs:
// - C_g   : Cost associated with regions of high confidence matches [H' x W' x (u, v, d, cost)]
// - C_b   : Cost associated with refions of invalid disparities [H' x W' x (u, v, cost)]
// - S_it  : Sparse support pixels with valid depths [N x (u, v, d)]
// - param : Parameter struct
//
// outputs:
// - S_it  : Updated sparse support pixels with valid depths
// #############################################################################
// This function loops over C_g and C_b. The pixels and the correspondendt depths
// in C_g are written as new support points to S_it.
// The pixels from C_b are collected in X and passed to the epipolar search. This 
// returns the matching pixels of the right image and the correspondendt depths.
// Those new found matches are then also added to S_it.
void support_resampling(cv::Mat &C_g, cv::Mat &C_b, 
                            cv::Mat &S_it, parameters &param,
							cv::Mat &I_l, cv::Mat &I_r){

	std::cout << "support_resampling.cpp" << std::endl;


	// define container for re-sampled or detected support pixels with unknown depths
	//int sz_X[] = {1, 2}; 
	std::cout << "support_resampling here -1" << std::endl;
	//cv::Mat X (2, sz_X, CV_32F, cv::Scalar::all(0));
	
	
	std::cout << "support_resampling here 0" << std::endl;
	// get grid size
	const int H_bar = int(param.H / param.sz_occ);
	const int W_bar = int(param.W / param.sz_occ); 

	cv::Mat X;
	X = cv::Mat(H_bar * W_bar, 2, CV_32F, 0.0);
	int X_length = 0;

	cv::Mat S_add;
	S_add = cv::Mat(H_bar * W_bar, 3, CV_32F, 0.0);
	int S_add_length = 0;
	// counters
	int count_X = 0, count_S_it = 0, count_epi = 0; 
	std::cout << "support_resampling here 1" << std::endl;
	for (int i_bar = 0; i_bar < H_bar; ++i_bar){
		for (int j_bar = 0; j_bar < W_bar; ++j_bar){

			if (C_b.at<float>(i_bar, j_bar, 0) != 0){
				// store (u,v) for bad point for resampling
				std::cout << "support_resampling here // store (u,v) for bad point for resampling" << std::endl;
				std::cout << C_b.at<float>(i_bar, j_bar, 0) << "/" << C_b.at<float>(i_bar, j_bar, 1) << std::endl;
				X.at<float>(X_length, 0) = C_b.at<float>(i_bar, j_bar, 0);
				X.at<float>(X_length, 1) = C_b.at<float>(i_bar, j_bar, 1);
				X_length++;

				//cv::Mat Pt = cv::Mat::zeros(1, 2, CV_32F);
				//Pt.at<float>(0, 0) = C_b.at<float>(i_bar, j_bar, 0);
	 			//Pt.at<float>(0, 1) = C_b.at<float>(i_bar, j_bar, 1);
 
				// add bad point to X
				//X.push_back(Pt);
				count_X++;
			}

			if (C_g.at<float>(i_bar, j_bar, 0) != 0){
				// store (u,v,d) for valid points
				std::cout << "support_resampling here // store (u,v,d) for valid points" << std::endl;
				std::cout << C_g.at<float>(i_bar, j_bar, 0) << "/" << C_g.at<float>(i_bar, j_bar, 1) << "/" << C_g.at<float>(i_bar, j_bar, 2) << std::endl;
				S_add.at<float>(S_add_length, 0) = C_g.at<float>(i_bar, j_bar, 0);
				S_add.at<float>(S_add_length, 1) = C_g.at<float>(i_bar, j_bar, 1);
				S_add.at<float>(S_add_length, 2) = C_g.at<float>(i_bar, j_bar, 2);

				S_add_length++;


				/*
				cv::Mat Pt = cv::Mat(1, 3, CV_32F);
				std::cout << "pushback?" << std::endl;
				S_it.push_back(Pt);

				std::cout << "mat?" << std::endl;
				S_it.at<float>(S_it.rows-1, 0) = C_g.at<float>(i_bar, j_bar, 0);
				S_it.at<float>(S_it.rows-1, 1) = C_g.at<float>(i_bar, j_bar, 1);
				S_it.at<float>(S_it.rows-1, 2) = C_g.at<float>(i_bar, j_bar, 2);
				*/
				
				// add valid support point to S_it_next
				//S_it.push_back(Pt);
				count_S_it++;
			}
		}
	}

	

	std::cout << "support_resampling here 2" << std::endl;
// ###
// epipolar search
// ###
	cv::Mat S_epi;
	S_epi = cv::Mat(X_length, 3, CV_32F, 0.0);
	int S_epi_length = 0;


	// pad a frame around the images
	int border = 2;
	cv::Mat I_r_p = cv::Mat(I_r.rows + border*2, I_r.cols + border*2, I_r.depth());
	cv::Mat I_l_p = cv::Mat(I_l.rows + border*2, I_l.cols + border*2, I_l.depth());
	cv::copyMakeBorder(I_r, I_r_p, border, border, border, border, cv::BORDER_REPLICATE);
	cv::copyMakeBorder(I_l, I_l_p, border, border, border, border, cv::BORDER_REPLICATE);

	std::cout << "support_resampling here 3" << std::endl;
	// get length of X
	int nuOpt = X.rows;
	// loop over X, leave first entry out
	for (int i=0; i < X_length; ++i){
		int d = -1;
		epipolar_search(I_l_p, I_r_p,
						int(X.at<float>(i, 0)), int(X.at<float>(i, 1)), d, param);

		S_epi.at<float>(i, 0) = X.at<float>(i, 0);
		S_epi.at<float>(i, 1) = X.at<float>(i, 1);
		S_epi.at<float>(i, 2) = float(d);

/*
		cv::Mat Pt_m = cv::Mat::zeros(1, 3, CV_32F);
		Pt_m.at<float>(0, 0) = X.at<float>(i, 0);
		Pt_m.at<float>(0, 1) = X.at<float>(i, 1);
		Pt_m.at<float>(0, 2) = float(d);

		// add valid support point to S_it_next
		//S_it.push_back(Pt_m);
		*/



		count_epi++;
	}


	// combine S_it, S_add and S_epi
	cv::Mat S_next;
	S_next = cv::Mat(S_it.rows + S_add_length + X_length, 3, CV_32F);
	for (int i = 0; i < S_it.rows + S_add_length + X_length; ++i){
		if (i < S_it.rows){
			S_next.at<float>(i, 0) = S_it.at<float>(i, 0);
			S_next.at<float>(i, 1) = S_it.at<float>(i, 1);
			S_next.at<float>(i, 2) = S_it.at<float>(i, 2);
		} else if(i < S_it.rows + S_add_length){
			S_next.at<float>(i, 0) = S_add.at<float>(i - S_it.rows, 0);
			S_next.at<float>(i, 1) = S_add.at<float>(i - S_it.rows, 1);
			S_next.at<float>(i, 2) = S_add.at<float>(i - S_it.rows, 2);
		} else{
			S_next.at<float>(i, 0) = S_epi.at<float>(i - S_it.rows - X_length, 0);
			S_next.at<float>(i, 1) = S_epi.at<float>(i - S_it.rows - X_length, 1);
			S_next.at<float>(i, 2) = S_epi.at<float>(i - S_it.rows - X_length, 2);
		}
	}
	S_it = S_next; 


	std::cout << count_X << " points stored for epi-search." << std::endl;
	std::cout << count_S_it << " points stored directly as support points." << std::endl;
	std::cout << count_epi << " points stored from the epi-search." << std::endl;

}


// this sub-routinr takes the padded left and right image, a pixelcoordinate u/v,
// a container for the disparity d and the parameter struct.
// It searches along the epipolar line (horizontal) and calculates for every pixel in the
// right image the census transform and the hemming distance to the census transform of the pixel
// u/v in the left image. The output is the disparity to the pixel with the smallest hemming cost.
void epipolar_search(cv::Mat &I_l_p, cv::Mat &I_r_p,
                        int u, int v, int & d, parameters &param){
	
	// get padded indices
	int u_p = u + 2;
	int v_p = v + 2;

	// get census of keypoint
	std::bitset<24> cens_l(0);
	census(I_l_p, u_p, v_p, cens_l);

	// set best cost to high and best indices to zero
	double best_cost = 1.5;
	int v_best = 0;

	// loop along the epipolar line
	for (int v_ = 0; v_<I_r_p.cols; ++v_){

		// get census of current pixel
		std::bitset<24> cens_c(0);
		census(I_r_p, u_p, v_ + 2, cens_c);

		// calculate error
		ushort cost = 0;
		std::bitset<24> mask(1);
		std::bitset<24> result(0);

		// calculating Hemming distance
		result = cens_l ^ cens_c;
		for (int it = 0; it < 24; ++it){
			std::bitset<24> current_bit = (result & (mask<<it))>>it;
			if (current_bit == mask){
				cost++;
			}
		}

		// normalize cost
		double n_cost = cost / 24.0;

		// if error < best_error : keep v_
		if (n_cost < best_cost){
			best_cost = n_cost;
			v_best = v_;
		}
	}

	// unpad v_best
	v_best = v_best -2;

	// calculate disparity with v_left - v_right
	d = v - v_best;
}