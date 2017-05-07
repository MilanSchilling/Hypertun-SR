#include "disparity_refinement.hpp"
#include <iostream>

// disparity refinement:
// inputs:
// -D_it : Interpolated disparity [H x W]
// -C_it : Normalized cost associated to D_it [H x W]
// -G    : Matrix G which points to the corresponding triangle for every pixel [H x W]
// -param: Parameter struct
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

	std::cout << "disparity_refinement.cpp" << std::endl;

	// loop over C_it
	for (int v = 0; v < param.H; ++v){
		for (int u = 0; u < param.W; ++u){

			if (G.at<int>(v,u) != -1){
				// Establish occupancy grid for resampled points
				int u_bar = int(std::floor(u / param.sz_occ));
				int v_bar = int(std::floor(v / param.sz_occ));
				if (u_bar >= param.W_bar) u_bar = param.W_bar - 1;
				if (v_bar >= param.H_bar) v_bar = param.H_bar - 1; 

				// If matching cost is lower than previous best final cost
				if (C_it.at<float>(v,u) < C_f.at<float>(v,u)){
					// store current disparity and cost to final
					D_f.at<float>(v,u) = D_it.at<float>(v,u);
					C_f.at<float>(v,u) = C_it.at<float>(v,u);
				}

				// If matching cost is lower than previous best valid cost
				if ((C_it.at<float>(v,u) < param.t_lo) && (C_it.at<float>(v,u) < C_g.at<float>(v_bar,u_bar,3))){

					assert(0 <= C_it.at<float>(v, u) && C_it.at<float>(v, u) <= 1);
					C_g.at<float>(v_bar,u_bar, 0) = u;
					C_g.at<float>(v_bar,u_bar, 1) = v;
					C_g.at<float>(v_bar,u_bar, 2) = D_it.at<float>(v,u);
					C_g.at<float>(v_bar,u_bar, 3) = C_it.at<float>(v,u);
					assert(0 <= C_g.at<float>(v_bar, u_bar, 3) && C_g.at<float>(v_bar, u_bar, 3) <= 1);
				}

				// If matching cost is higher than previous worst invalid cost
				float c_it_cur = C_it.at<float>(v,u);
				float c_b_cur = C_b.at<float>(v_bar, u_bar, 2);
				if ((c_it_cur > param.t_hi) && (c_it_cur > c_b_cur)){

					assert(0 <= C_it.at<float>(v, u) && C_it.at<float>(v, u) <= 1);
					C_b.at<float>(v_bar,u_bar, 0) = u;
					C_b.at<float>(v_bar,u_bar, 1) = v;
					C_b.at<float>(v_bar,u_bar, 2) = C_it.at<float>(v,u);
					assert(0 <= C_b.at<float>(v_bar, u_bar, 2) && C_b.at<float>(v_bar, u_bar, 2) <= 1);
					std::cout << "disparity_refinement: added point to C_b." << std::endl;
					std::cout << "C_it.at<float>(i,j) = " << C_it.at<float>(v,u) << std::endl;
					
				}
			}
		}
	}
}