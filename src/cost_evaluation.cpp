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
// - C_it: Cost associated to D_it 
void cost_evaluation(cv::Mat &I_l, cv::Mat &I_r, cv::Mat &D_it, cv::Mat &C_it){
	// TODO: census container to bool container!

	std::bitset<24> cens(0);
	std::bitset<24> mask(1);
	std::bitset<24> bla(1);
	bla = bla | (mask<<1);
	bla = bla | (mask<<6);


	for (int i = 0; i < 24; ++i){
		if (i%2 == 0){
			cens = cens | (mask<<i);
		}
	}
 
	std::cout << cens << std::endl;
	std::cout << bla << std::endl;


	int c = 0;
	std::bitset<24> result(0);
	result = cens & bla;
	std::cout << result << std::endl;
	std::cout << "***" << std::endl;
	for (int i = 0; i < 24; ++i){
		std::bitset<24> bli;
		bli = (result & (mask<<i))>>i;
		std::cout << bli << std::endl;
		if (bli == mask){
			c++;
		}
	}
	std::cout << c << std::endl;

	std::cout << "cost_evaluation.cpp" << std::endl;


	// Get image height and width
	int H = I_l.rows;
	int W = I_r.cols;

	// pad the images
	int border = 2;
	cv::Mat I_r_p = cv::Mat(I_r.rows + border*2, I_r.cols + border*2, I_r.depth());
	cv::Mat I_l_p = cv::Mat(I_l.rows + border*2, I_l.cols + border*2, I_l.depth());
	cv::copyMakeBorder(I_r, I_r_p, border, border, border, border, cv::BORDER_REPLICATE);
	cv::copyMakeBorder(I_l, I_l_p, border, border, border, border, cv::BORDER_REPLICATE);

	// loop over interpolated disparities
	for (int i = 0; i < H; ++i){
		for (int j = 0; j < W; j++){

			double disp = D_it.at<int>(i,j); // TODO: verify order of i,j!

			//evaluate cost for given disparity
			int i_pad = i + border;
			int j_pad = j + border;

			//cv::Mat census_l = cv::Mat::zeros(1, 24, CV_8U);
			//cv::Mat census_r = cv::Mat::zeros(1, 24, CV_8U);
			std::bitset<24> cens_l(0);
			std::bitset<24> cens_r(0);
			int census_left = 0;
			int census_right = 0;
			//short int census_l [24] = {};
			//short int census_r [24] = {};

			// get census of left image
			census(I_l_p, i_pad, j_pad, cens_l);

			// get census of right image at the interpolated disparity
			census(I_r_p, i_pad, j_pad + disp, cens_r);

			// calculate cost
			ushort cost = 0;
			std::bitset<24> mask(1); 
			for (int it = 0; it < 24; ++it){
				//if ()
				//if (*(census_l+it) == *(census_r+it))
					//cost = cost + 1;
			}

			// write cost to C_it
			C_it.at<int>(i,j) = cost;

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
					//census.at<int>(0,count) = 1;
					count++;
				} else{
					// else leave at zero
					count++;
				}
			}
		}
	}
	//return census;
}