#include "disparity_interpolation.hpp"
#include <iostream>


// inputs:
// - G   : HxW Matrix with an index for each pixel to the corresponding triangle
// - T   : 4 x num_riangles matrix with the plane parameters for each pixel
//
// outputs:
// - D_it: HxW matrix with an interpolated disparity for each pixel
void disparity_interpolation(cv::Mat &G, cv::Mat &T, cv::Mat &D_it){
	
	std::cout << "disparity_interpolation.cpp" << std::endl;

	// get image dimensions
	int H = G.rows;
	int W = G.cols;

	// loop over every pixel
	for (int i = 0; i < H; ++i) {
		for (int j = 0; j < W; ++j) {

			// get index of the corresponding triangle 
			int idx_trig = G.at<int>(i,j);

			// get plane parameters
			if (idx_trig >= 0) {
				float a = T.at<float>(0,idx_trig);
				float b = T.at<float>(1,idx_trig);
				float c = T.at<float>(2,idx_trig);
				float d = T.at<float>(3,idx_trig);

				// interpolate disparity
				D_it.at<float>(i,j) = (d - a*j - b*i) / c;

				//std::cout << D_it.at<float>(i,j) << " ";
			}
		}
		//std::cout << std::endl;
	}	
}