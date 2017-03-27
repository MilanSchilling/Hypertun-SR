#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sparse_stereo.hpp"
#include "delaunay_triangulation.hpp"
#include "disparity_interpolation.hpp"
#include "cost_evaluation.hpp"
#include "disparity_refinement.hpp"
#include "support_resampling.hpp"

struct parameters {
	// Occupancy grid size used for re-sampling
	int sz_occ;

	// Number of iterations the algorithm is allowed to run
	int n_iters;

	// Lower and upper threshold for validating disparities
	double t_lo;
	double t_hi;
};

void pipeline() {

	std::cout << "pipeline.cpp" << std::endl;

	//Load parameters
	parameters param;
	param.sz_occ = 32;
	param.n_iters = 1;
	param.t_lo = 1; // placeholder, verify optimal value
	param.t_hi = 5; // placeholder, verify optimal value

	// Load images
	cv::Mat I_l = cv::imread("../data/data_scene_flow/testing/image_2/000000_10.png");
	cv::Mat I_r = cv::imread("../data/data_scene_flow/testing/image_3/000000_10.png");
	
	// Display images
	cv::imshow("Image Left", I_l);
	cv::imshow("Image Right", I_r);
	cv::waitKey(0);

	// Get image height and width
	int H = I_l.rows;
	int W = I_r.cols;

	// Initialize final disparity and associated cost
	cv::Mat D_f = cv::Mat(H, W, CV_64F, 0.0);
	cv::Mat C_f = cv::Mat(H, W, CV_64F, param.t_hi);

	// Declare other variables
	cv::Mat S; // set of N support points with valid depths
	cv::Mat G; // graph (3D plane parameters?) from delaunay triangulation
	cv::Mat D; // dense piece-wise planar disparity
	cv::Mat C; // cost associated to D
	cv::Mat C_g; // cost associated with regions of good matches
	cv::Mat C_b; // cost associated with regions of bad matches

	sparse_stereo();
	delaunay_triangulation();

	for (int i = 0; i < param.n_iters; ++i) {
		disparity_interpolation();
		cost_evaluation();
		disparity_refinement(D_f, C_f, C_f); // matrices are passed for debug resons
		if (i != param.n_iters) {
			support_resampling();
			delaunay_triangulation();
		}
	}
	


}