#include "sparse_stereo.hpp"

using namespace std;
using namespace cv;
using namespace sparsestereo;
using namespace boost;
using namespace boost::posix_time;

void sparse_stereo(cv::Mat I_l, cv::Mat I_r, cv::Mat &S){

	std::cout << "sparse_stereo.cpp" << std::endl;

	// Sign input images
	cv::Mat_<unsigned char> leftImg, rightImg;
	leftImg=I_l;
	rightImg=I_r;
	
	// Stereo matching parameters
	double uniqueness = 0.7;
	int maxDisp = 70;
	int leftRightStep = 2;

	// Feature detection parameters
	double adaptivity = 1.0;
	int minThreshold = 10;

	// Determine the height and width
	int original_height=leftImg.rows;
	int original_width=leftImg.cols;

	// the standard image size for exFAST
	int new_width=640;
	int new_height=480;
	Size size(new_width,new_height);

	// Resize the input images to the standard image size
	cv::Mat_<unsigned char> leftImg_resize, rightImg_resize;
	resize(leftImg, leftImg_resize, size);
	resize(rightImg, rightImg_resize, size);

	// no rectification data in case of KITTI dataset
	StereoRectification* rectification = NULL;

	// The stereo matcher. SSE Optimized implementation is only available for a 5x5 window
	SparseStereo<CensusWindow<5>, short> stereo(maxDisp, 1, uniqueness, rectification, false, false, leftRightStep);

	// Feature detectors for left and right image
	FeatureDetector* leftFeatureDetector = new ExtendedFAST(true, minThreshold, adaptivity, false, 2);			 
	FeatureDetector* rightFeatureDetector = new ExtendedFAST(false, minThreshold, adaptivity, false, 2);

	ptime lastTime = microsec_clock::local_time();
	vector<SparseMatch> correspondences;

	// Objects for storing final and intermediate results
	cv::Mat_<char> charLeft(leftImg_resize.rows, leftImg_resize.cols), 
	charRight(rightImg_resize.rows, rightImg_resize.cols);
	Mat_<unsigned int> censusLeft(leftImg_resize.rows, leftImg_resize.cols), 
	censusRight(rightImg_resize.rows, rightImg_resize.cols);
	vector<KeyPoint> keypointsLeft, keypointsRight;

	// For performance evaluation we do the stereo matching 100 times
	for(int i=0; i< 100; i++) {

		// Featuredetection. This part can be parallelized with OMP
		#pragma omp parallel sections default(shared) num_threads(2)
		{
			#pragma omp section
			{
				ImageConversion::unsignedToSigned(leftImg_resize, &charLeft);
				Census::transform5x5(charLeft, &censusLeft);
				keypointsLeft.clear();
				leftFeatureDetector->detect(leftImg_resize, keypointsLeft);
			}
		
			#pragma omp section
			{
				ImageConversion::unsignedToSigned(rightImg_resize, &charRight);
				Census::transform5x5(charRight, &censusRight);
				keypointsRight.clear();
				rightFeatureDetector->detect(rightImg_resize, keypointsRight);
			}
		}

		// Stereo matching. Not parallelized (overhead too large)
		correspondences.clear();
		stereo.match(censusLeft, censusRight, keypointsLeft, keypointsRight, &correspondences);
	}

	// Print statistics
	time_duration elapsed = (microsec_clock::local_time() - lastTime);
	cout << "Time for 100x stereo matching: " << elapsed.total_microseconds()/1.0e6 << "s" << endl
		<< "Features detected in left image: " << keypointsLeft.size() << endl
		<< "Features detected in right image: " << keypointsRight.size() << endl
		<< "Percentage of matched features: " << (100.0 * correspondences.size() / keypointsLeft.size()) << "%" << endl;

	// Save matched feature coordinates and corresponding disparity in an array 
	// Back transformation into orginal frame
	double correspondences_disparity_original[correspondences.size()][3];	
	Point mark[correspondences.size()];
	
	for(int j=0; j<(int)correspondences.size(); j++) { 
		correspondences_disparity_original[j][0] = correspondences[j].imgLeft->pt.x/new_width*original_width;
		correspondences_disparity_original[j][1] = correspondences[j].imgLeft->pt.y/new_height*original_height;
		correspondences_disparity_original[j][2] = correspondences[j].disparity()/new_width*original_width;
			
		mark[j].x=correspondences_disparity_original[j][0];
		mark[j].y=correspondences_disparity_original[j][1];
	}

	// Highlight matches as colored boxes
	Mat_<Vec3b> screen(leftImg.rows, leftImg.cols);
	cvtColor(leftImg, screen, CV_GRAY2BGR);
		
	for(int i=0; i<(int)correspondences.size(); i++) {
		double scaledDisp = (double)correspondences_disparity_original[i][2] / maxDisp;
		Vec3b color;
		if(scaledDisp > 0.5)
			color = Vec3b(0, (1 - scaledDisp)*512, 255);
		else color = Vec3b(0, 255, scaledDisp*512);

		circle(screen, mark[i], 1, (Scalar) color, 3);
	}

	// Display image and wait
	namedWindow("Stereo");
	imshow("Stereo", screen);
	waitKey();

	// Fill set of support points
	int num_points = correspondences.size();
	S = cv::Mat(num_points, 3, CV_32F, 0.0);
	for (int i = 0; i < num_points; ++i)
	{
		S.at<float>(i,0) = correspondences_disparity_original[i][0];
		S.at<float>(i,1) = correspondences_disparity_original[i][1];
		S.at<float>(i,2) = correspondences_disparity_original[i][2];
	}


	// Clean up
	delete leftFeatureDetector;
	delete rightFeatureDetector;

}
