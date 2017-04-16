#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <sstream>

#include "sparse_stereo.hpp"
#include "delaunay_triangulation.hpp"
#include "disparity_interpolation.hpp"
#include "cost_evaluation.hpp"
#include "disparity_refinement.hpp"
#include "support_resampling.hpp"
#include "parameters.hpp"


void line2(cv::Mat& img, const cv::Point& start, const cv::Point& end, 
                     const cv::Scalar& c1,   const cv::Scalar& c2) {
    cv::LineIterator iter(img, start, end, 8);

    for (int i = 0; i < iter.count; i++, iter++) {
       double alpha = double(i) / iter.count;
       // note: using img.at<T>(iter.pos()) is faster, but 
       // then you have to deal with mat type and channel number yourself
       img(cv::Rect(iter.pos(), cv::Size(1, 1))) = c1 * (1.0 - alpha) + c2 * alpha;
    }
}

void showGrid(cv::Mat &I_l, cv::Mat &S, cv::Mat &E, std::string str){
	// Draw Triangles and display image
	cv::Mat I_triangles = I_l;
	cv::cvtColor(I_triangles, I_triangles, CV_GRAY2RGB);
	/*for (int i = 0; i < S.cols; ++i) {
		cv::circle(I_triangles, cv::Point(S.at<float>(0,i),S.at<float>(1,i)), 
			1, cv::Scalar(0,255,255),CV_FILLED, 1,0);
	}*/
	int k = 0;
	for (int i = 0; i < E.rows/2; ++i) {
		int i1 = E.at<int>(k++,0);
		int i2 = E.at<int>(k++,0);
		cv::Point p1(S.at<float>(i1,0), S.at<float>(i1,1));
		cv::Point p2(S.at<float>(i2,0), S.at<float>(i2,1));

		float maxDisp = 64.0; // staehlii: THIS IS ACTUALLY NOT A CONSTANT BUT I DON'T WANT TO SEARCH FOR THE CORRECT VALUE
		float scaledDisp1 = S.at<float>(i1,2)/maxDisp;
		float scaledDisp2 = S.at<float>(i2,2)/maxDisp;
		cv::Vec3b color1;
		cv::Vec3b color2;
		if(scaledDisp1 < 0.5)
			color1 = cv::Vec3b(0, scaledDisp1*512, 255);
		else color1 = cv::Vec3b(0, 255, (1-scaledDisp1)*512);

		if(scaledDisp2 < 0.5)
			color2 = cv::Vec3b(0, scaledDisp2*512, 255);
		else color2 = cv::Vec3b(0, 255, (1-scaledDisp2)*512);
		line2(I_triangles, p1, p2, (cv::Scalar) color1, (cv::Scalar) color2);
		//std::cout << "drew line: " << i1 << ", " << i2 << std::endl;
	}
	cv::imshow(str, I_triangles);
	cv::waitKey(0);
}


void pipeline() {

	std::cout << "Using CV version: " << CV_VERSION << std::endl;
	std::cout << "pipeline.cpp" << std::endl;

	//Load parameters
	parameters param;
	param.sz_occ = 32;
	param.n_iters = 3;
	param.t_lo = 0.05; // placeholder, verify optimal value
	param.t_hi = 0.7; // placeholder, verify optimal value

	// Load images
	cv::Mat I_l = cv::imread("../data/data_scene_flow/testing/image_2/000000_10.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat I_r = cv::imread("../data/data_scene_flow/testing/image_3/000000_10.png", CV_LOAD_IMAGE_GRAYSCALE);
	
	// Display images
	//cv::imshow("Image Left", I_l);
	//cv::imshow("Image Right", I_r);
	//cv::waitKey(0);

	// Get image height and width
	param.H = I_l.rows;
	param.W = I_r.cols;

	// divide image into discrete parts
	int H_bar = int(param.H / param.sz_occ);
	int W_bar = int(param.W / param.sz_occ);


	// Initialize final disparity and associated cost
	cv::Mat D_f = cv::Mat(param.H, param.W, CV_64F, 0.0);
	cv::Mat C_f = cv::Mat(param.H, param.W, CV_64F, param.t_hi);

	// Declare other variables
	cv::Mat S; // set of N support points with valid depths, Nx3 with [u,v,d]
	cv::Mat G; // graph: corresponding triangle of each pixel from delaunay triangulation
	cv::Mat T; // Triangle 4D plane parameters from delaunay triangulation
	cv::Mat E; // Triangle edges for plotting
	cv::Mat D; // dense piece-wise planar disparity
	cv::Mat C; // cost associated to D

	int sz_g[] = {H_bar, W_bar, 4}; // dimension of C_g
	int sz_b[] = {H_bar, W_bar, 3}; // dimension of C_b
	cv::Mat C_g (3, sz_g, CV_64F, cv::Scalar::all(0)); // cost associated with regions of good matches
	cv::Mat C_b (3, sz_b, CV_64F, cv::Scalar::all(0)); // cost associated with regions of bad matches

	// write thresholds to C_g and C_b
	// TODO: do this within inizialisation above!
	for (int i = 0; i < H_bar; ++i){
		for (int j = 0; j < W_bar; ++j){
			C_g.at<double>(i,j,3) = param.t_lo;
			C_b.at<double>(i,j,2) = param.t_hi;
		}
	}

	cv::Mat D_it = cv::Mat(param.H, param.W, CV_64F, 0.0); // Intermediate disparity (interpolated)
	cv::Mat C_it = cv::Mat(param.H, param.W, CV_64F, param.t_hi);; // Cost associated to D_it

	// Create dummy variable to show functionality
	/*float S_array[3][8] = {100, 100, 200, 200, 0, 0, 300, 300, 
						   100, 200, 100, 200, 0, 300, 0, 300,
						   500, 500, 500, 500, 200, 200, 200, 200};*/

	float S_array[3][8] = {100, 100, 500,
						   100, 200, 500,
						   200, 100, 500,
						   200, 200, 500,
						   0, 0, 200,
						   0, 300, 200,
						   300, 0, 200,
						   300, 300, 200};

	cv::Mat S2 = cv::Mat(8, 3, CV_32F, S_array);


	sparse_stereo(I_l, I_r, S);
	delaunay_triangulation(S, param.H, param.W, G, T, E);
	std::cout << "Rows of S: " << S.rows << std::endl;
	std::cout << "Rows of E: " << E.rows << std::endl;
	showGrid(I_l, S, E, "Delaunay 1");


	for (int i = 0; i < param.n_iters; ++i) {
		disparity_interpolation(G, T, D_it);
		cost_evaluation(I_l, I_r, D_it, C_it);
		disparity_refinement(D_it, C_it, D_f, C_f, C_g, C_b, param);

		if (i != param.n_iters) {
			support_resampling(C_g, C_b, S, param, I_l, I_r);
			// empty E ?
			cv::Mat E;
			delaunay_triangulation(S, param.H, param.W, G, T, E);
			std::cout << "Rows of S: " << S.rows << std::endl;
			std::cout << "Rows of E: " << E.rows << std::endl;
			std::ostringstream oss;
			oss << "Delaunay " << i+2;
			std::string str = oss.str();
			showGrid(I_l, S, E, str);
		}
	}
	
	

}

