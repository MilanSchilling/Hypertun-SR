#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sparse_stereo.hpp"
#include "delaunay_triangulation.hpp"
#include "disparity_interpolation.hpp"
#include "cost_evaluation.hpp"
#include "disparity_refinement.hpp"
#include "support_resampling.hpp"


void pipeline(){

	cv::Mat imgL = cv::imread("../data/data_scene_flow/testing/image_2/000000_10.png");
	cv::Mat imgR = cv::imread("../data/data_scene_flow/testing/image_3/000000_10.png");
	cv::imshow("Image Left", imgL);
	cv::imshow("Image Right", imgR);
	cv::waitKey(0);
	std::cout << "Test" << std::endl;

	sparse_stereo();
	delaunay_triangulation();
	disparity_interpolation();
	cost_evaluation();
	disparity_refinement();
	support_resampling();


}