#include "cost_evaluation.hpp"
#include <iostream>
#include <bitset>

// cost_evaluation:
// inputs: 
// - I_l : image left
// - I_r : image right
// - D_it: Interpolated disparity
//
// outputs:
// - C_it: Normalized cost associated to D_it
// ############################################
// This function compares every patch with the correspondent patch, 
// given the interpolated disparity, using a census comparison.
// It returns a matirx containing the costs for every pixel. 
void cost_evaluation(cv::Mat &I_l, cv::Mat &I_r, cv::Mat &D_it, cv::Mat &C_it){

	std::cout << "cost_evaluation.cpp" << std::endl;

	// Get image height and width
	int H = I_l.rows;
	int W = I_r.cols;

	// pad a frame around the images
	int border = 2;
	cv::Mat I_r_p = cv::Mat(I_r.rows + border*2, I_r.cols + border*2, I_r.depth());
	cv::Mat I_l_p = cv::Mat(I_l.rows + border*2, I_l.cols + border*2, I_l.depth());
	cv::copyMakeBorder(I_r, I_r_p, border, border, border, border, cv::BORDER_REPLICATE);
	cv::copyMakeBorder(I_l, I_l_p, border, border, border, border, cv::BORDER_REPLICATE);

	// loop over interpolated disparities
	for (int i = 0; i < H; ++i){
		for (int j = 0; j < W; j++){

			float disp = D_it.at<float>(i,j); // TODO: verify order of i,j!

			//evaluate cost for given disparity
			int i_pad = i + border;
			int j_pad = j + border;

			std::bitset<24> cens_l(0);
			std::bitset<24> cens_r(0);

			// get census of left image
			census(I_l_p, i_pad, j_pad, cens_l);

			// get census of right image at the interpolated disparity
			census(I_r_p, i_pad, j_pad + disp, cens_r);

			// calculate cost
			ushort cost = 0;
			std::bitset<24> mask(1);
			std::bitset<24> result(0);

			// calculating Hemming distance
			result = cens_l ^ cens_r;
			for (int it = 0; it < 24; ++it){
				std::bitset<24> current_bit = (result & (mask<<it))>>it;
				if (current_bit == mask){
					cost++;
				}
			}

			// normalize cost
			float n_cost = cost / 24.0;

			// write cost to C_it
			C_it.at<float>(i,j) = n_cost;
		}
	}
}


// This helper takes a padded image and a pixel coordinate and the container for the census as argument
// It creates a binary descriptor for the neighborhood around the pixel, writes 1 if a pixel is smaller than the center.§§
void census (cv::Mat &paddedImg, int i_pad, int j_pad, std::bitset<24> &census){
	//cv::Mat census = cv::Mat::zeros(1, 24, CV_8U);

	// get intensity of center pixel
	uchar center = paddedImg.at<uchar>(i_pad, j_pad);

	int count = 0;

	std::bitset<24> mask(1);

	// loop over window
	for (int a = 0; a < 5; ++a){
		for (int b = 0; b < 5; ++b){
			// leave the center pixel out
			if  (!((a == 0) && (b == 0))){
				// get the value of the neighbor pixel
				uchar px = paddedImg.at<uchar>(i_pad - 2 + a, j_pad -2 + b);
				// set census to 1 if intenity is less
				if (px < center){
					census = census | (mask<<count);
					count++;
				} else{
					// else leave at zero
					count++;
				}
			}
		}
	}
}