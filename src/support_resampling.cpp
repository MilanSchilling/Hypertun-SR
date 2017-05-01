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


	// get number of points in C_b and C_g
	int noBadPts = 0;
	int noGoodPts = 0;

	for (int it_u=0; it_u<param.W_bar; ++it_u){
		for (int it_v=0; it_v<param.H_bar; ++it_v){
			
			if (C_b.at<float>(it_v, it_u, 2) != param.t_hi){
				//std::cout << "it_u/it_v = " << it_u << "/" << it_v << std::endl;
				//std::cout << "C_b(it_v,it_u) = " << C_b.at<float>(it_v, it_u, 0) << "/" << C_b.at<float>(it_v, it_u, 1) << "/" << C_b.at<float>(it_v, it_u, 2) << std::endl;
				noBadPts++;
			}
				
			
			if (C_g.at<float>(it_v, it_u, 3) < param.t_lo){
				//std::cout << "it_u/it_v = " << it_u << "/" << it_v << std::endl;
				//std::cout << "C_g(it_v,it_u) = " << C_g.at<float>(it_v, it_u, 0) << "/" << C_g.at<float>(it_v, it_u, 1) << "/" << C_g.at<float>(it_v, it_u, 2) << "/" << C_g.at<float>(it_v, it_u, 3) << std::endl;
				noGoodPts++;
			}
				
		}
	}


	int sz_X[] = {noBadPts, 2};
	cv::Mat X = cv::Mat(2, sz_X, CV_32F, cv::Scalar::all(0));
	int X_length = 0;





	int sz_add[] = {noGoodPts, 3};
	cv::Mat S_add = cv::Mat(2, sz_add, CV_32F,cv::Scalar::all(0));
	int S_add_length = 0;
	// counters
	int count_X = 0;
	int count_S_it = 0;
	int count_epi = 0; 


	for (int v_bar = 0; v_bar < param.H_bar; ++v_bar){
		for (int u_bar = 0; u_bar < param.W_bar; ++u_bar){



			if (C_b.at<float>(v_bar, u_bar, 2) > param.t_hi){
				// store (u,v) for bad point for resampling
				assert(0 <= C_b.at<float>(v_bar, u_bar, 0) && C_b.at<float>(v_bar, u_bar, 0) <= 1242);
				assert(0 <= C_b.at<float>(v_bar, u_bar, 1) && C_b.at<float>(v_bar, u_bar, 1) <= 375);
				assert(0 <= C_b.at<float>(v_bar, u_bar, 2) && C_b.at<float>(v_bar, u_bar, 2) <= 1);
				assert(0 <= C_g.at<float>(v_bar, u_bar, 3) && C_b.at<float>(v_bar, u_bar, 3) <= 1);

				//std::cout << "support_resampling here // store (u,v,c) for bad point for resampling" << std::endl;
				//std::cout << C_b.at<float>(v_bar, u_bar, 0) << "/" << C_b.at<float>(v_bar, u_bar, 1) << "/" << C_b.at<float>(v_bar, u_bar, 2) << std::endl;
				X.at<float>(X_length, 0) = C_b.at<float>(v_bar, u_bar, 0);
				X.at<float>(X_length, 1) = C_b.at<float>(v_bar, u_bar, 1);
				X_length++;

				count_X++;
			}

			if (C_g.at<float>(v_bar, u_bar, 3) < param.t_lo){
				// store (u,v,d) for valid points
				assert(0 <= C_g.at<float>(v_bar, u_bar, 0) && C_g.at<float>(v_bar, u_bar, 0) <= 1242);
				assert(0 <= C_g.at<float>(v_bar, u_bar, 1) && C_g.at<float>(v_bar, u_bar, 1) <= 375);

				// be sure costs are between 0 and 1
				//std::cout << "v_bar / u_bar = " << v_bar << "/" << u_bar << std::endl;
				assert(0 <= C_b.at<float>(v_bar, u_bar, 2) && C_b.at<float>(v_bar, u_bar, 2) <= 1);
				assert(0 <= C_g.at<float>(v_bar, u_bar, 3) && C_b.at<float>(v_bar, u_bar, 3) <= 1);

				//std::cout << "support_resampling here // store (u,v,d,c) for valid points" << std::endl;
				//std::cout << C_g.at<float>(v_bar, u_bar, 0) << "/" << C_g.at<float>(v_bar, u_bar, 1) << "/" << C_g.at<float>(v_bar, u_bar, 2) << "/" << C_g.at<float>(v_bar, u_bar, 3) << std::endl;
				S_add.at<float>(S_add_length, 0) = C_g.at<float>(v_bar, u_bar, 0);
				S_add.at<float>(S_add_length, 1) = C_g.at<float>(v_bar, u_bar, 1);
				S_add.at<float>(S_add_length, 2) = C_g.at<float>(v_bar, u_bar, 2);

				S_add_length++;
				
				// add valid support point to S_it_next
				//S_it.push_back(Pt);
				count_S_it++;
			}
		}
	}

	

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


	// loop over X, leave first entry out
	for (int i=0; i < X_length; ++i){
		int d = -1;
		assert(0 <= int(X.at<float>(i, 0)) && int(X.at<float>(i, 0)) <= 1242);
		assert(0 <= int(X.at<float>(i, 1)) && int(X.at<float>(i, 1)) <= 375);
		epipolar_search(I_l_p, I_r_p,
						int(X.at<float>(i, 0)), int(X.at<float>(i, 1)), d, param);

		assert(d >= 0);

		S_epi.at<float>(i, 0) = X.at<float>(i, 0);
		S_epi.at<float>(i, 1) = X.at<float>(i, 1);
		S_epi.at<float>(i, 2) = float(d);

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
			//std::cout << "i global is " << i << " and i from S_it is " << i << std::endl;
		} else if(i < S_it.rows + S_add_length){
			S_next.at<float>(i, 0) = S_add.at<float>(i - S_it.rows, 0);
			S_next.at<float>(i, 1) = S_add.at<float>(i - S_it.rows, 1);
			S_next.at<float>(i, 2) = S_add.at<float>(i - S_it.rows, 2);
			//std::cout << "i global is " << i << " and i from S_add is " << i - S_it.rows << std::endl;
		} else{
			S_next.at<float>(i, 0) = S_epi.at<float>(i - S_it.rows - S_add_length, 0);
			S_next.at<float>(i, 1) = S_epi.at<float>(i - S_it.rows - S_add_length, 1);
			S_next.at<float>(i, 2) = S_epi.at<float>(i - S_it.rows - S_add_length, 2);
			//std::cout << "i global is " << i << " and i from S_epi is " << i - S_it.rows - S_add_length << std::endl;
		}
	}
	S_it = S_next.clone(); 


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

	assert(0 <= u && u <= 1242);
	assert(0 <= v && v <= 375);

	
	// get padded indices
	int u_p = u + 2;
	int v_p = v + 2;

	assert(0 <= u_p && u_p <= 1242);
	assert(0 <= v_p && v_p <= 375);

	// get census of keypoint
	std::bitset<24> cens_l(0);
	//std::cout << "epi1" << std::endl;
	//std::cout << "u_p/v_p = " << u_p << "/" << v_p << std::endl;
	assert(v_p > 0);
	census(I_l_p, u_p, v_p, cens_l);

	// set best cost to high and best indices to zero
	double best_cost = 1.5;
	int u_best = 0;

	// loop along the epipolar line
	for (int u_ = 0; u_ <= u_p - 2; ++u_){

		// get census of current pixel
		std::bitset<24> cens_c(0);
		census(I_r_p, u_ + 2, v_p, cens_c);

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
		float n_cost = cost / 24.0;

		// if error < best_error : keep v_
		if (n_cost < best_cost){
			best_cost = n_cost;
			u_best = u_;
		}
	}

	// unpad u_best
	//u_best = u_best - 2;

	// calculate disparity with u_left - u_right
	d = u - u_best;
}