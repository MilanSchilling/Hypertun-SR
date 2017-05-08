using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <png++/png.hpp>

#include "pipeline.hpp"

int main() {
	
	//setting folder paths and choosing all .png files
	cv::String path_left("../data/data_scene_flow/training/image_2/*.png");
	cv::String path_right("../data/data_scene_flow/training/image_3/*.png");
	cv::String path_disp("../data/data_scene_flow/training/disp_occ_0/*.png");
	
	//adressing filenames with indexes
	vector<cv::String> filenames_left;
	vector<cv::String> filenames_right;
	vector<cv::String> filenames_disp;
	cv::glob(path_left, filenames_left);
	cv::glob(path_right, filenames_right);
	cv::glob(path_disp, filenames_disp);

	//iterate over all files
	for (size_t i=0; i<filenames_left.size(); i++) {

		//reading in left and right images in grayscale
		cv::Mat I_l = cv::imread(filenames_left[i], CV_LOAD_IMAGE_GRAYSCALE);
		cv::Mat I_r = cv::imread(filenames_right[i], CV_LOAD_IMAGE_GRAYSCALE);

		cv::Mat D_f; //iInitialize final disparity		

		// calling
		pipeline(I_l, I_r, D_f);
	
		cout << "################################################" << endl;
		cout << "ACCURACY (comparing with ground-truth disparity)" << endl;
		cout << "################################################" << endl;
	
		// disparity maps are saved as uint16 PNG images, can be opened with the libpng++ library
		png::image<png::gray_pixel_16> D_gt_png(filenames_disp[i]);
		int32_t D_gt_png_width=D_gt_png.get_width();
		int32_t D_gt_png_height=D_gt_png.get_height();
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
				gt_value = D_gt_png.get_pixel(u,v);
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

		cout << "less than 2 pixels: " << counter_2/n_loops * 100 << " %" << endl;
		cout << "less than 3 pixels: " << counter_3/n_loops * 100 << " %" << endl;
		cout << "less than 4 pixels: " << counter_4/n_loops * 100 << " %" << endl;
		cout << "less than 5 pixels: " << counter_5/n_loops * 100 << " %" << endl;

	}

	return 0;

}
