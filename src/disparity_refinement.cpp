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
							cv::Mat &G,
							cv::Mat &D_f, cv::Mat &C_f,
							cv::Mat &C_g, cv::Mat &C_b,
							parameters &param){
	// TODO: pass parameter struct to this function
	std::cout << "disparity_refinement.cpp" << std::endl;

	// loop over C_it
	for (int i = 0; i < param.H; ++i){
		for (int j = 0; j < param.W; ++j){

			if (G.at<float>(i,j) != -1){
				// Establish occupancy grid for resampled points
				int i_bar = int(i / param.sz_occ);
				int j_bar = int(j / param.sz_occ); 

				// If matching cost is lower than previous best final cost
				if (C_it.at<float>(i,j) < C_f.at<float>(i,j)){
					// store current disparity and cost to final
					D_f.at<float>(i,j) = D_it.at<float>(i,j);
					C_f.at<float>(i,j) = C_it.at<float>(i,j);
				}

				// If matching cost is lower than previous best valid cost
				if ((C_it.at<float>(i,j) < param.t_lo) && (C_it.at<float>(i,j) < C_g.at<float>(i_bar,j_bar,3))){
					C_g.at<float>(i_bar,j_bar, 0) = i;
					C_g.at<float>(i_bar,j_bar, 1) = j;
					C_g.at<float>(i_bar,j_bar, 2) = D_it.at<float>(i,j);
					C_g.at<float>(i_bar,j_bar, 3) = C_it.at<float>(i,j);
					
					std::cout << "disparity_refinement: added point to C_g." << std::endl;
					std::cout << "C_it.at<float>(i,j) = " << C_it.at<float>(i,j) << std::endl;
				}

				// If matching cost is higher than previous worst invalid cost
				/*if ((C_it.at<float>(i,j) >= param.t_hi) && (C_it.at<float>(i,j) > C_b.at<float>(i_bar, j_bar, 2))){
					C_b.at<float>(i_bar,j_bar, 0) = i;
					C_b.at<float>(i_bar,j_bar, 1) = j;
					C_b.at<float>(i_bar,j_bar, 2) = C_it.at<float>(i,j);
					//std::cout << "disparity_refinement: added point to C_b." << std::endl;
					//std::cout << "C_it.at<float>(i,j) = " << C_it.at<float>(i,j) << std::endl;
				}*/
			}

			
		}
	}
}