#include "pipeline_sgbm.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


int main() {

	const int ACCURACY_DATASET = 0;
	const int PERFORMANCE_DATASET = 1;

	int DATASET = ACCURACY_DATASET;

	cv::String path_left;
	cv::String path_right;
	cv::String path_disp;

	//setting folder paths and choosing all .png files
	if (DATASET == ACCURACY_DATASET) {
		path_left = "../data/data_stereo_flow/training/colored_0/*.png";
		path_right = "../data/data_stereo_flow/training/colored_1/*.png";
		path_disp = "../data/data_stereo_flow/training/disp_occ/*.png";
	}
	if (DATASET == PERFORMANCE_DATASET) {
		path_left = "../data/data_odometry_color/00/image_2/*.png";
		path_right = "../data/data_odometry_color/00/image_3/*.png";
		path_disp = "../data/data_odometry_color/00/image_3/*.png";
	}
	
	//adressing filenames with indexes
	std::vector<cv::String> filenames_left;
	std::vector<cv::String> filenames_right;
	std::vector<cv::String> filenames_disp;
	cv::glob(path_left, filenames_left);
	cv::glob(path_right, filenames_right);
	cv::glob(path_disp, filenames_disp);
	int num_files = filenames_left.size();

	bool single_image = false;
	if (single_image) num_files = 1;

	for (size_t i=0; i<num_files; i++) {
		if (single_image) i = 9;
		
		boost::posix_time::ptime time_start = boost::posix_time::microsec_clock::local_time();

		if (DATASET == PERFORMANCE_DATASET) pipeline_sgbm(filenames_left[i*2], filenames_right[i*2], " ");
		else pipeline_sgbm(filenames_left[i*2], filenames_right[i*2], filenames_disp[i]);

		boost::posix_time::time_duration time_elapsed = (boost::posix_time::microsec_clock::local_time() - time_start);
		std::cout << "#################################################" << std::endl;
		std::cout << std::setprecision(2);
		std::cout << "PIPELINE TOOK: " << time_elapsed.total_microseconds()/1.0e6 << " seconds" << std::endl; 
		std::cout << "WITH A SPEED OF: " << 1.0e6/time_elapsed.total_microseconds() << " Hz" << std::endl;
		std::cout << "#################################################" << std::endl;

		if (DATASET == ACCURACY_DATASET) cv::waitKey(0);
		if (DATASET == PERFORMANCE_DATASET) cv::waitKey(1);
	}

	return 0;

}