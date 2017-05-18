#include "pipeline.hpp"
#include "stats.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


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


	const int ALL = 0;
	const int ONE = 1;
	bool single_image = false;

	
	//#######################
	int range = 5;               //<<<<-------------- // set here range to 'ALL', 'ONE' or a specific number
	//#######################

	// prepare range of images
	if (range == ALL){
		// loop over all images
		range = num_files;
	} else
	if (range == ONE){
		// just do one image
		range = 1;
		single_image = true;
	}

	// create instance of stats
	stats statistics;

	// container for average numbers
	float avrg_PL_time = 0;
	float avrg_PL_freq = 0;
	float avrg_ALG_time = 0;
	float avrg_ALG_freq = 0;

	for (size_t i=0; i<range; i++) {
		if (single_image) i = 9;
		
		boost::posix_time::ptime time_start = boost::posix_time::microsec_clock::local_time();

		pipeline(filenames_left[i*2], filenames_right[i*2], filenames_disp[i], statistics);

		boost::posix_time::time_duration time_elapsed = (boost::posix_time::microsec_clock::local_time() - time_start);

		avrg_PL_time += time_elapsed.total_microseconds()/1.0e3;
		avrg_PL_freq += 1.0e6/time_elapsed.total_microseconds();
		avrg_ALG_time += statistics.alg_time;
		avrg_ALG_freq += statistics.alg_freq;

		std::cout << "#################################################" << std::endl;
		std::cout << std::setprecision(3);
		std::cout << std::setw(50) << std::left << "PIPELINE TOOK: " << std::right << time_elapsed.total_microseconds()/1.0e3 << " ms" << std::endl; 
		std::cout << std::setw(50) << std::left << "WITH A SPEED OF: " << std::right << 1.0e6/time_elapsed.total_microseconds() << " Hz" << std::endl;
		std::cout << "#################################################" << std::endl;

		cv::waitKey(0);
	}

	if (range != 1){
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::setw(50) << std::left << "ANALYSIS OVER IMAGES:" << std::right << range << std::endl;
		std::cout << std::setw(50) << std::left << "NUMBER OF ITERATIONS:" << std::right << statistics.it << std::endl;
		std::cout << std::endl;
		std::cout << std::setprecision(3);
		std::cout << std::setw(50) << std::left << "PIPELINE TOOK ON AVERAGE: " << std::right << avrg_PL_time / range << " ms" << std::endl;
		std::cout << std::setw(50) << std::left << "WITH AN AVERAGE SPEED OF: " << std::right << avrg_PL_freq / range << " Hz" << std::endl;
		std::cout << std::endl;
		std::cout << std::setw(50) << std::left << "ALGORITHM TOOK ON AVERAGE: " << std::right << avrg_ALG_time / range << " ms" << std::endl;
		std::cout << std::setw(50) << std::left << "WITH AN AVERAGE SPEED OF: " << std::right << avrg_ALG_freq / range << " Hz" << std::endl;
		std::cout << "#################################################" << std::endl;
		std::cout << "#################################################" << std::endl;


	}

	return 0;

}