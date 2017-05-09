#include "pipeline.hpp"
#include <opencv2/core/core.hpp>


int main() {

	//setting folder paths and choosing all .png files
	cv::String path_left("../data/data_stereo_flow/training/colored_0/*.png");
	cv::String path_right("../data/data_stereo_flow/training/colored_1/*.png");
	cv::String path_disp("../data/data_stereo_flow/training/disp_occ/*.png");
	
	//adressing filenames with indexes
	std::vector<cv::String> filenames_left;
	std::vector<cv::String> filenames_right;
	std::vector<cv::String> filenames_disp;
	cv::glob(path_left, filenames_left);
	cv::glob(path_right, filenames_right);
	cv::glob(path_disp, filenames_disp);
	int num_files = filenames_left.size();

	bool single_image = true;
	if (single_image) num_files = 1;

	for (size_t i=0; i<num_files; i++) {
		if (single_image) i = 9;
		pipeline(filenames_left[i*2], filenames_right[i*2], filenames_disp[i]);
	}

	return 0;

}