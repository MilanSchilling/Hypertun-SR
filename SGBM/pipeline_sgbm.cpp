// Adapted from:
// http://www.jayrambhia.com/blog/disparity-mpas
// https://github.com/jayrambhia/Vision/blob/master/OpenCV/C%2B%2B/disparity.cpp


#include "pipeline_sgbm.hpp"

using namespace cv;
using namespace std;

void computeAccuracy(cv::Mat D_f, cv::String filename_disp);

void pipeline_sgbm(cv::String filename_left, cv::String filename_right, cv::String filename_disp) {

	cv::Mat I_l = cv::imread(filename_left);
	cv::Mat I_r = cv::imread(filename_right);

	boost::posix_time::ptime algorithm_time_start = boost::posix_time::microsec_clock::local_time();

	Mat g1, g2;
	Mat disp, disp8;

	cvtColor(I_l, g1, CV_BGR2GRAY);
	cvtColor(I_r, g2, CV_BGR2GRAY);

	StereoSGBM sgbm;
	sgbm.SADWindowSize = 5;
	sgbm.numberOfDisparities = 256; // Adapted this value to use whole range of uint8
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = 0;			// Adapted this value to avoid segmentation fault
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = false;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;

	sgbm(g1, g2, disp);
	normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

	boost::posix_time::time_duration algorithm_time_elapsed = (boost::posix_time::microsec_clock::local_time() - algorithm_time_start);
	std::cout << "************************************************" << std::endl;
	std::cout << std::setprecision(2);
	std::cout << "ALGORITHM TOOK: " << algorithm_time_elapsed.total_microseconds()/1.0e6 << " seconds" << std::endl; 
	std::cout << "WITH A SPEED OF: " << 1.0e6/algorithm_time_elapsed.total_microseconds() << " Hz" << std::endl;
	std::cout << "************************************************" << std::endl;

	cv::imshow("Disparity", disp8);

	computeAccuracy(disp8, filename_disp);

}

void computeAccuracy(cv::Mat D_f, cv::String filename_disp){

	std::cout << "################################################" << std::endl;
	std::cout << "ACCURACY (comparing with ground-truth disparity)" << std::endl;
	std::cout << "################################################" << std::endl;

	cv::Mat D_gt_png = cv::imread(filename_disp);

	int D_gt_png_width = D_f.cols;
	int D_gt_png_height = D_f.rows;

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

			gt_value = D_gt_png.at<uint16_t>(v,u,0);

			if (D_f.at<uint8_t>(v,u,0) != 0 && gt_value != 0) { // both ground-truth and estimate disparity valid
				float disp = D_f.at<uint8_t>(v,u,0)*1;
				n_loops = n_loops + 1;	
				// ground-truth disparity computed by dividing pixel value by 256 
				if (abs(disp-gt_value/256.0) < threshold_5)  
					counter_5 = counter_5 + 1;
					if (abs(disp-gt_value/256.0) < threshold_4)
						counter_4 = counter_4 + 1;
						if (abs(disp-gt_value/256.0) < threshold_3)
							counter_3 = counter_3 + 1;
							if (abs(disp-gt_value/256.0) < threshold_2)
								counter_2 = counter_2 + 1;
			}
		}
	}

	std::cout << "less than 2 pixels: " << counter_2/n_loops * 100 << " %" << std::endl;
	std::cout << "less than 3 pixels: " << counter_3/n_loops * 100 << " %" << std::endl;
	std::cout << "less than 4 pixels: " << counter_4/n_loops * 100 << " %" << std::endl;
	std::cout << "less than 5 pixels: " << counter_5/n_loops * 100 << " %" << std::endl;

}