#include "cost_evaluation.hpp"
#include "sparsestereo/sparsestereo-inl.h"
#include "sparsestereo/census-inl.h"
#include "sparsestereo/censuswindow.h"
#include "sparsestereo/imageconversion.h"
#include <iostream>
#include <bitset>

// cost_evaluation:
// inputs: 
// - I_l : image left [H x W]
// - I_r : image right [H x W]
// - D_it: Interpolated disparity [H x W]
// - G   : Matrix G which points to the corresponding triangle for every pixel [H x W]
// - O   : [N x (u,v)] matrix, containing all N pixels with high gradient
// - param: parameter struct
//
// outputs:
// - C_it  : Normalized cost associated to D_it
// - census_l: Census transformed left image
// - census_r:  Census transformed right image
// ############################################
// This function compares every patch with the correspondent patch, 
// given the interpolated disparity, using a census comparison.
// It returns a matirx containing the costs for every pixel. 
void cost_evaluation(cv::Mat &I_l, cv::Mat &I_r, 
                        cv::Mat &D_it, cv::Mat &G,
                         cv::Mat & O, parameters &param,
                         cv::Mat &C_it){



	// try with exfast census
	//####################################
	std::cout << "cost_evaluation.cpp census try" << std::endl;

	cv::Mat empt_cens = cv::Mat(param.H, param.W, CV_32S, cv::Scalar(-1));

	// Sign input images
	cv::Mat_<unsigned char> leftImg, rightImg;
	leftImg = I_l.clone();
	rightImg = I_r.clone();

	// create container for char images
	cv::Mat_<char> charLeft(param.H, param.W);
	cv::Mat_<char> charRight(param.H, param.W);

	// create container for census outputs
	cv::Mat_<unsigned int> censusLeft(param.H, param.W);
	cv::Mat_<unsigned int> censusRight(param.H, param.W);

	// convert images
	sparsestereo::ImageConversion::unsignedToSigned(leftImg, &charLeft);
	sparsestereo::ImageConversion::unsignedToSigned(rightImg, &charRight);
	//std::cout << "conversion done" << std::endl;

	// perform census transform
	sparsestereo::Census::transform5x5(charLeft, &censusLeft);
	sparsestereo::Census::transform5x5(charRight, &censusRight);

	// loop over all high gradient pixels
	for (int it = 0; it < param.nOfHiGradPix; it++){

		// extract u,v
		int u = O.at<int>(it, 0);
		int v = O.at<int>(it, 1);

		// check if triangle is defined for this pixel
		if (G.at<int>(v,u) != -1){
			// get interpolated disparity
			int disp = D_it.at<float>(v,u);
			if (u + disp > 1242){
				disp = disp - (u + disp - 1242);
			}
			assert(u + disp <= 1242);

			sparsestereo::HammingDistance mHamming;
			// calculate hamming distance
			//std::cout << "caluclate hamming at (u+d,v) = (" << u << "+" << (int)disp << "," << v <<  ")" << std::endl;

			unsigned int currCensusLeft = censusLeft.at<unsigned int>(v,u);
			//std::cout << "extracted census Left:  " << std::bitset<24>(currCensusLeft) << std::endl;
			unsigned int currCensusRight = censusRight.at<unsigned int>(v,u + (int)disp);
			//std::cout << "extracted census Right: " << std::bitset<24>(currCensusRight) << std::endl;
			unsigned char hamming = mHamming.calculate(currCensusLeft, currCensusRight);
			// TODO: verify (v+disp, u) or (v, u+disp)
			//std::cout << "hamming = " << int(hamming) << std::endl;

			/*
			if(hamming == 24){
				empt_cens.at<int>(v,u) = 1;
			}
			*/

			// normalize cost
			float n_cost = hamming / 24.0;
			//std::cout << "n_cost: " << n_cost << std::endl;
			// write cost to C_it
			C_it.at<float>(v,u) = n_cost;
		}
	}

	/*
	// loop over interpolated disparities
	for (int v = 0; v < param.H; ++v){
		for (int u = 0; u < param.W; u++){
			
			// check if triangle is defined for this pixel
			if (G.at<int>(v,u) != -1){
				// get interpolated disparity
				int disp = D_it.at<float>(v,u);
				if (u + disp > 1242){
					disp = disp - (u + disp - 1242);
				}
				assert(u + disp <= 1242);

				sparsestereo::HammingDistance mHamming;
				// calculate hamming distance
				//std::cout << "caluclate hamming at (u+d,v) = (" << u << "+" << (int)disp << "," << v <<  ")" << std::endl;

				unsigned int currCensusLeft = censusLeft.at<unsigned int>(v,u);
				//std::cout << "extracted census Left:  " << std::bitset<24>(currCensusLeft) << std::endl;
				unsigned int currCensusRight = censusRight.at<unsigned int>(v,u + (int)disp);
				//std::cout << "extracted census Right: " << std::bitset<24>(currCensusRight) << std::endl;
				unsigned char hamming = mHamming.calculate(currCensusLeft, currCensusRight);
				// TODO: verify (v+disp, u) or (v, u+disp)
				//std::cout << "hamming = " << int(hamming) << std::endl;


				// normalize cost
				float n_cost = hamming / 24.0;
				//std::cout << "n_cost: " << n_cost << std::endl;
				// write cost to C_it
				C_it.at<float>(v,u) = n_cost;
			}
		}
	}
	*/

	// copy census transformed images for output
	census_l = censusLeft;
	census_r = censusRight;

	/*
	// show where census is zero
	cv::Mat cens = I_l.clone();
	cv::cvtColor(cens, cens, CV_GRAY2RGB);
	cv::Vec3b color = cv::Vec3b(0,0,255);
	// loop over empt_cens
	for (int v = 0; v < param.H; ++v){
		for (int u = 0; u < param.W; u++){
			if(empt_cens.at<int>(v,u) > 0){
				cens.at<cv::Vec3b>(v,u) = color;
			}

		}
	}

	cv::imshow("empty census", cens);
	cv::waitKey(0);
	*/



	/*
	std::cout << "cost_evaluation.cpp" << std::endl;

	// pad a frame around the images
	int border = 2;
	cv::Mat I_r_p = cv::Mat(param.H + border*2, param.W + border*2, I_r.depth());
	cv::Mat I_l_p = cv::Mat(param.H + border*2, param.W + border*2, I_l.depth());
	cv::copyMakeBorder(I_r, I_r_p, border, border, border, border, cv::BORDER_REPLICATE);
	cv::copyMakeBorder(I_l, I_l_p, border, border, border, border, cv::BORDER_REPLICATE);

	// loop over interpolated disparities
	for (int v = 0; v < param.H; ++v){
		for (int u = 0; u < param.W; u++){

			if (G.at<int>(v,u) != -1){
				float disp = D_it.at<float>(v,u); // TODO: verify order of i,j!

				//evaluate cost for given disparity
				int u_pad = u + border;
				int v_pad = v + border;

				std::bitset<24> cens_l(0);
				std::bitset<24> cens_r(0);

				// get census of left image
				census(I_l_p, u_pad, v_pad, cens_l);

				// get census of right image at the interpolated disparity
				census(I_r_p, u_pad, v_pad + disp, cens_r);

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
				//std::cout << "cost: " << cost << std::endl;
				// write cost to C_it
				C_it.at<float>(v,u) = n_cost;
				//std::cout << "n_cost: " << n_cost << std::endl;
			}
			
		}
	}
	*/
}


// This helper takes a padded image and a pixel coordinate and the container for the census as argument
// It creates a binary descriptor for the neighborhood around the pixel, writes 1 if a pixel is smaller than the center.§§
void census (cv::Mat &paddedImg, int u_pad, int v_pad, std::bitset<24> &census){
	//cv::Mat census = cv::Mat::zeros(1, 24, CV_8U);

	// get intensity of center pixel
	uchar center = paddedImg.at<uchar>(v_pad, u_pad);

	int count = 0;

	std::bitset<24> mask(1);

	// loop over window
	for (int a = 0; a < 5; ++a){
		for (int b = 0; b < 5; ++b){
			// leave the center pixel out
			if  (!((a == 0) && (b == 0))){
				// get the value of the neighbor pixel
				uchar px = paddedImg.at<uchar>(v_pad - 2 + a, u_pad -2 + b);
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