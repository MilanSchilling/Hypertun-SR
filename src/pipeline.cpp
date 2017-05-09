

#include <iostream> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <sstream>
#include <cassert>

#include "sparse_stereo.hpp"
#include "delaunay_triangulation.hpp"
#include "disparity_interpolation.hpp"
#include "cost_evaluation.hpp"
#include "disparity_refinement.hpp"
#include "support_resampling.hpp"
#include "parameters.hpp"

// header of 'line2'
void line2(cv::Mat& img, const cv::Point& start, const cv::Point& end, 
                     const cv::Scalar& c1,   const cv::Scalar& c2);
// header of 'showGrid'
void showGrid(cv::Mat I_l, cv::Mat S, cv::Mat E, std::string str);

// header of 'showG'
void showG (cv::Mat I_l, cv::Mat G, parameters param, std::string str);

// header of 'showDisparity'
void showDisparity(cv::Mat I_l, cv::Mat D_it, std::string str);


// header of 'showSupportPts'
void showSupportPts(cv::Mat I_l, cv::Mat S_it, std::string str);

// header of ´computeAccuracy'
void computeAccuracy(cv::Mat D_f, cv::String filename_disp);


void pipeline(cv::String filename_left, cv::String filename_right, cv::String filename_disp) {
	std::cout << "#######" << std::endl;
	std::cout << "DATASET: http://www.cvlibs.net/datasets/kitti/eval_stereo_flow.php?benchmark=flow" << std::endl;
	std::cout << "#######" << std::endl << std::endl;
	std::cout << "Using CV version: " << CV_VERSION << std::endl;
	std::cout << "pipeline.cpp" << std::endl;

	//Load parameters
	parameters param;
	param.sz_occ = 32;
	param.n_iters = 3;
	param.t_lo = 2.f/24; // placeholder, verify optimal value
	param.t_hi = 24.f/24; // placeholder, verify optimal value
	param.im_grad = 50;

	// Load images
	cv::Mat I_l = cv::imread(filename_left, CV_LOAD_IMAGE_COLOR);
	cv::Mat I_r = cv::imread(filename_right, CV_LOAD_IMAGE_COLOR);
	
	// crop image to be dividable by 16
	int offset_u = 5;
	int offset_v = 2;
	cv::Rect roi; // region of interest
	roi.x = offset_u;
	roi.y = offset_v;
	roi.width = 1216;
	roi.height = 368;

	cv::Mat I_l_c = I_l(roi);
	cv::Mat I_r_c = I_r(roi);
	cv::Mat I_l_cg, I_r_cg;

	// apply gaussian blur
	cv::GaussianBlur(I_l_c, I_l_c, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	cv::GaussianBlur(I_r_c, I_r_c, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

	// Convert to gray
	cv::cvtColor(I_l_c, I_l_cg, CV_BGR2GRAY);
	cv::cvtColor(I_r_c, I_r_cg, CV_BGR2GRAY);

	// Generate grad_x and grad_y
  	cv::Mat grad_l_x, grad_l_y;
  	cv::Mat abs_grad_l_x, abs_grad_l_y;
	cv::Mat grad_l;

	// parameters for Gradient
	int scale = 1;
	int delta = 0;
	int ddepth = -1;

	// Gradient X
	cv::Sobel( I_l_cg, grad_l_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(grad_l_x, abs_grad_l_x);

	// Gradient Y
	cv::Sobel( I_l_cg, grad_l_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
 	cv::convertScaleAbs(grad_l_y, abs_grad_l_y );	

	// Total Gradient (approximate)
	cv::addWeighted(abs_grad_l_x, 0.5, abs_grad_l_y, 0.5, 0, grad_l);

	int highGradCount = 0; 
	// count how many pixels with high gradient in left image
	for (int vv = 0; vv < I_l_cg.rows; vv++){
		for (int uu = 0; uu < I_l_cg.cols; uu++){
			if(int(grad_l.at<uchar>(vv,uu)) > param.im_grad){
				highGradCount++;
			}
		}
	}
	// TODO: get rid of this loop above, it is just to count how many pixel above threshold. (maybe push_back?)

	param.nOfHiGradPix = highGradCount;
	std::cout << "Number of pixels with high gradient: " << param.nOfHiGradPix << std::endl;

	// container for high gradient pixel values [nuOfHiGradPixels x (u,v)]
	cv::Mat O = cv::Mat(param.nOfHiGradPix, 2, CV_32S, 0.0);
	highGradCount = 0;
	for (int vv = 0; vv < I_l_cg.rows; vv++){
		for (int uu = 0; uu < I_l_cg.cols; uu++){
			if(int(grad_l.at<uchar>(vv,uu)) > param.im_grad){
				O.at<int>(highGradCount, 0) = uu;
				O.at<int>(highGradCount, 1) = vv;
				highGradCount++;
			}
		}
	}

	// show gradient image
	//cv::imshow("left gradient", grad_l);
	//cv::waitKey(0);

	// Get image height and width
	param.H = I_l_cg.rows;
	param.W = I_r_cg.cols;

	// Get initial grid height and width
	param.H_bar = std::floor(param.H / param.sz_occ);
	param.W_bar = std::floor(param.W / param.sz_occ);

	// Initialize final disparity and associated cost
	cv::Mat D_f = cv::Mat(param.H, param.W, CV_32F, 0.0);
	cv::Mat C_dummy_2 = cv::Mat(param.H, param.W, CV_32F, 0.0); // dummy array
	cv::Mat C_f = cv::Mat(param.H, param.W, CV_32F, param.t_hi);

	// Declare other variables
	cv::Mat S; // set of N support points with valid depths, Nx3 with [u,v,d]
	cv::Mat G; // graph: corresponding triangle of each pixel from delaunay triangulation
	cv::Mat T; // Triangle 4D plane parameters from delaunay triangulation
	cv::Mat E; // Triangle edges for plotting
	cv::Mat D; // dense piece-wise planar disparity
	cv::Mat C; // cost associated to D
	cv::Mat C_g; // cost associated with regions of good matches
	cv::Mat C_b; // cost associated with regions of bad matches 
	cv::Mat D_it; // Intermediate disparity (interpolated)
	cv::Mat C_it; // Cost associated to D_it
	cv::Mat census_l; // census transformed left image
	cv::Mat census_r; // census transformed right image


	// execute 'sparse_stereo' with elapsed time estimation 
	boost::posix_time::ptime lastTime = boost::posix_time::microsec_clock::local_time();
	sparse_stereo(I_l_cg, I_r_cg, S);
	boost::posix_time::time_duration elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << "Elapsed Time for 'sparse_stereo': " << elapsed.total_microseconds()/1.0e6 << " s" << std::endl;

	// execute 'delaunay_triangulation' with elapsed time estimation
	lastTime = boost::posix_time::microsec_clock::local_time();
	delaunay_triangulation(S, param.H, param.W, G, T, E);
	elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
	std::cout << "Elapsed Time for 'delaunay_triangulation': " << elapsed.total_microseconds()/1.0e6 << " s" << std::endl;

	// show matrix G
	//showG(I_l, G, param, "G after delaunay");
	
	// set all support points in G to -1
	for (int j=0; j<S.rows; ++j){
		int u = S.at<float>(j,0);
		int v = S.at<float>(j,1);
		G.at<int>(v,u) = -1;
	}
	// TODO: set G = -1 for all supportpoints within delaunay!
	
	// show the grid from the delaunay triangulation
	showGrid(I_l_cg, S, E, "Delaunay 1");
	boost::posix_time::ptime algorithm_time_start = boost::posix_time::microsec_clock::local_time();

	for (int i = 0; i < param.n_iters; ++i) {
		std::cout << "################################################" << std::endl;
		std::cout << "ITERATION :: " << i << std::endl;
		std::cout << "################################################" << std::endl;

		// initialize D_it and C_it new for every iteration
		D_it = cv::Mat(param.H, param.W, CV_32F, 0.0);
		C_it = cv::Mat(param.H, param.W, CV_32F, param.t_hi);

		// execute 'disparity_interpolation' with elapsed time estimation
		lastTime = boost::posix_time::microsec_clock::local_time();
		disparity_interpolation(G, T, O, param, D_it);
		elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
		std::cout << "Elapsed Time for 'disparity_interpolation': " << elapsed.total_microseconds()/1.0e6 << " s" << std::endl;
		
		// show matrix G
		//showG(I_l, G, param, "G");

		// show disparity from disparity_interpolation
		//showDisparity(I_l_c, D_it, "Disparity interpolated");

		
		// execute 'cost_evaluation' with elapsed time estimation
		lastTime = boost::posix_time::microsec_clock::local_time();
		cost_evaluation(I_l_cg, I_r_cg, D_it, G, O, param, C_it, census_l, census_r);
		elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
		std::cout << "Elapsed Time for 'cost_evaluation': " << elapsed.total_microseconds()/1.0e6 << " s" << std::endl;	

		// initialize C_g and C_b new for every iteration
		int sz_g[] = {param.H_bar, param.W_bar, 4}; // dimension of C_g
		int sz_b[] = {param.H_bar, param.W_bar, 3}; // dimension of C_b
		C_g = cv::Mat(3, sz_g, CV_32F, cv::Scalar::all(0));
		cv::Mat C_dummy = cv::Mat(3, sz_g, CV_32F, cv::Scalar::all(0)); // dummy array
		C_b = cv::Mat(3, sz_b, CV_32F, cv::Scalar::all(0));

		// write thresholds to C_g and C_b
		for (int k = 0; k < param.W_bar; ++k){
			for (int l = 0; l < param.H_bar; ++l){
				C_g.at<float>(l,k,0) = 0.0;
				C_g.at<float>(l,k,1) = 0.0;
				C_g.at<float>(l,k,2) = 0.0;
				C_g.at<float>(l,k,3) = param.t_lo;

				C_b.at<float>(l,k,0) = 0.0;
				C_b.at<float>(l,k,1) = 0.0;
				C_b.at<float>(l,k,2) = param.t_hi;
			}
		}

		// execute 'disparity_refinement' with elapsed time estimation
		lastTime = boost::posix_time::microsec_clock::local_time();
		disparity_refinement(D_it, C_it, G, O, D_f, C_f, C_g, C_b, param);
		elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
		std::cout << "Elapsed Time for 'disparity_refinement': " << elapsed.total_microseconds()/1.0e6 << " s" << std::endl;
		
		// Prepare for next iteration, if not last iteration
		if (i != param.n_iters) {

			// execute 'support_resampling' with elapsed time estimation
			lastTime = boost::posix_time::microsec_clock::local_time();
			support_resampling(C_g, C_b, S, param, I_l_c, I_r_c, census_l, census_r);
			elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
			std::cout << "Elapsed Time for 'support_resampling': " << elapsed.total_microseconds()/1.0e6 << " s" << std::endl;
		
			// execute 'delaunay_triangulation' with elapsed time estimation
			lastTime = boost::posix_time::microsec_clock::local_time();
			delaunay_triangulation(S, param.H, param.W, G, T, E);
			elapsed = (boost::posix_time::microsec_clock::local_time() - lastTime);
			std::cout << "Elapsed Time for 'delaunay_triangulation': " << elapsed.total_microseconds()/1.0e6 << " s" << std::endl;

			// set all support points in G to -1
			for (int j=0; j<S.rows; ++j){
				int u = S.at<float>(j,0);
				int v = S.at<float>(j,1);
				G.at<int>(v,u) = -1;
			}
			// TODO: set G = -1 for all supportpoints within delaunay!

			// show matrix G
			//showG(I_l, G, param, "G7");

			// refine gridsize
			param.sz_occ = param.sz_occ / 2;

			// update grid height and width
			param.H_bar = int(std::floor(param.H / param.sz_occ));
			param.W_bar = int(std::floor(param.W / param.sz_occ));

			
			// show delaunay grid for i-th iteration
			std::ostringstream oss;
			oss << "Delaunay " << i+2;
			std::string str = oss.str();
			showGrid(I_l_cg, S, E, str);
			
		}
	}

	boost::posix_time::time_duration algorithm_time_elapsed = (boost::posix_time::microsec_clock::local_time() - algorithm_time_start);

	std::cout << "************************************************" << std::endl;
	std::cout << std::setprecision(2);
	std::cout << "ALGORITHM TOOK: " << algorithm_time_elapsed.total_microseconds()/1.0e6 << " seconds" << std::endl; 
	std::cout << "WITH A SPEED OF: " << 1.0e6/algorithm_time_elapsed.total_microseconds() << " Hz" << std::endl;
	std::cout << "************************************************" << std::endl;

	showGrid(I_l_cg, S, E, "final Delaunay");
	showSupportPts(I_l_cg, S, "final Support Points");
	showDisparity(I_l_cg, D_f, "final Disparity");
	computeAccuracy(D_f, filename_disp);

	cv::waitKey(0);
}




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



void showGrid(cv::Mat I_l, cv::Mat S, cv::Mat E, std::string str){
	// Draw Triangles and display image
	cv::Mat I_triangles = I_l.clone();
	cv::cvtColor(I_triangles, I_triangles, CV_GRAY2RGB);
	/*for (int i = 0; i < S.cols; ++i) {
		cv::circle(I_triangles, cv::Point(S.at<float>(0,i),S.at<float>(1,i)), 
			1, cv::Scalar(0,255,255),CV_FILLED, 1,0);
	}*/
	int k = 0;
	int E_end = E.rows/2;

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
}



void showG (cv::Mat I_l, cv::Mat G, parameters param, std::string str){

	std::cout << "showG" << std::endl;
	cv::Mat G_img = I_l.clone();
	

	// loop over all pixels
	for (int u = 0; u < param.W; ++u){
		for (int v = 0; v < param.H; ++v){

			//check if G(u,v) == -1
			if (G.at<int>(v, u, 0) == -1){ // G.at<int>(u, v) == -1
				uchar & color = G_img.at<uchar>(v,u);
				color = 0;
				G_img.at<uchar>(v,u) = color;

			} else{
				uchar & color = G_img.at<uchar>(v,u);
				color = 255;
				G_img.at<uchar>(v,u) = color;

			}
		}
	}

	cv::imshow(str, G_img);
	cv::waitKey(0);
}


void showDisparity(cv::Mat I_l, cv::Mat D_it, std::string str){
		cv::Mat disparity = I_l.clone();
		cv::cvtColor(disparity, disparity, CV_GRAY2RGB);

		for (int x = 0; x < I_l.rows; ++x){
			for (int y = 0; y < I_l.cols; ++y){
				float scaledDisp = D_it.at<float>(x,y)/64.0;
				cv::Vec3b color;
				cv::Point point;
				point.x = y;
				point.y = x;
				if(scaledDisp < 0.5)
					color = cv::Vec3b(0, scaledDisp*512, 255);
				else color = cv::Vec3b(0, 255, (1-scaledDisp)*512);

				if (scaledDisp != 0)
				circle(disparity, point, 1, (cv::Scalar) color, 1);
			}
		}

		cv::imshow(str, disparity);
}

void showSupportPts(cv::Mat I_l, cv::Mat S_it, std::string str){
	cv::Mat support = I_l.clone();
	cv::cvtColor(support, support, CV_GRAY2RGB);

	// loop over all points
	for (int i = 0; i < S_it.rows; ++i){
		cv::Point p1(S_it.at<float>(i,0), S_it.at<float>(i,1));
		float maxDisp = 64.0; // staehlii: THIS IS ACTUALLY NOT A CONSTANT BUT I DON'T WANT TO SEARCH FOR THE CORRECT VALUE
		float scaledDisp1 = S_it.at<float>(i,2)/maxDisp;
		cv::Vec3b color1;

		cv::circle(support, p1, 1, cv::Scalar(0, scaledDisp1*512, 255), 2, 8, 0);

		cv::imshow(str, support);
	}
}



void computeAccuracy(cv::Mat D_f, cv::String filename_disp){

	std::cout << "################################################" << std::endl;
	std::cout << "ACCURACY (comparing with ground-truth disparity)" << std::endl;
	std::cout << "################################################" << std::endl;

	cv::Mat D_gt_png = cv::imread(filename_disp);

	int D_gt_png_width = D_f.cols;
	int D_gt_png_height = D_f.rows;
	int offset_u = 5;
	int offset_v = 2;
	cv::Rect roi; // region of interest
	roi.x = offset_u;
	roi.y = offset_v;
	roi.width = 1216;
	roi.height = 368;

	cv::Mat D_gt_png_c = D_gt_png(roi);
	float gt_value;

	// initialize some parameters needed for calculation
	float n_loops = 0;
	float counter_2 = 0;
	float counter_3 = 0;
	float counter_4 = 0;
	float counter_5 = 0;
	float threshold_2 = 2;
	float threshold_3 = 3;
	float threshold_4 = 4;
	float threshold_5 = 5;
	
	// comparison with ground-truth disparity and 4 different threshold
	for (int32_t v=0; v<D_gt_png_height; v++) {
		for (int32_t u=0; u<D_gt_png_width; u++) {

			gt_value = D_gt_png_c.at<uint16_t>(v,u,0);

			if (D_f.at<float>(v,u) != 0 && gt_value != 0) { // both ground-truth and estimate disparity valid
				n_loops = n_loops + 1;	
				// ground-truth disparity computed by dividing pixel value by 256 
				if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_5)  
					counter_5 = counter_5 + 1;
					if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_4)
						counter_4 = counter_4 + 1;
						if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_3)
							counter_3 = counter_3 + 1;
							if (abs(D_f.at<float>(v,u)-gt_value/256.0) < threshold_2)
								counter_2 = counter_2 + 1;
			}
		}
	}

	std::cout << "less than 2 pixels: " << counter_2/n_loops * 100 << " %" << std::endl;
	std::cout << "less than 3 pixels: " << counter_3/n_loops * 100 << " %" << std::endl;
	std::cout << "less than 4 pixels: " << counter_4/n_loops * 100 << " %" << std::endl;
	std::cout << "less than 5 pixels: " << counter_5/n_loops * 100 << " %" << std::endl;
}