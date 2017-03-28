#include "sparse_stereo.hpp"

using namespace std;
using namespace cv;
using namespace sparsestereo;
using namespace boost;
using namespace boost::posix_time;

void sparse_stereo(cv::Mat I_l, cv::Mat I_r){

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

	// KITTI dataset already rectified
	StereoRectification* rectification = NULL;

	// The stereo matcher. SSE Optimized implementation is only available for a 5x5 window
	SparseStereo<CensusWindow<5>, short> stereo(maxDisp, 1, uniqueness, rectification, false, false, leftRightStep);

	// Feature detectors for left and right image
	FeatureDetector* leftFeatureDetector = new ExtendedFAST(true, minThreshold, adaptivity, false, 2);			
	FeatureDetector* rightFeatureDetector = new ExtendedFAST(false, minThreshold, adaptivity, false, 2);

	vector<SparseMatch> correspondences;

	// Objects for storing final and intermediate results
	cv::Mat_<char> charLeft(leftImg.rows, leftImg.cols), charRight(rightImg.rows, rightImg.cols);
	Mat_<unsigned int> censusLeft(leftImg.rows, leftImg.cols), censusRight(rightImg.rows, rightImg.cols);
	vector<KeyPoint> keypointsLeft, keypointsRight;

	for(int i=0; i< 100; i++) {
		// Featuredetection. This part can be parallelized with OMP
		#pragma omp parallel sections default(shared) num_threads(2)
		{
			#pragma omp section
			{
				ImageConversion::unsignedToSigned(leftImg, &charLeft);
				Census::transform5x5(charLeft, &censusLeft);
				keypointsLeft.clear();
				leftFeatureDetector->detect(leftImg, keypointsLeft);
			}

			#pragma omp section
			{
				ImageConversion::unsignedToSigned(rightImg, &charRight);
				Census::transform5x5(charRight, &censusRight);
				keypointsRight.clear();
				rightFeatureDetector->detect(rightImg, keypointsRight);
			}
		}
				
		// Stereo matching. Not parallelized (overhead too large)
		correspondences.clear();
		stereo.match(censusLeft, censusRight, keypointsLeft, keypointsRight, &correspondences);
	}

	// Highlight matches as colored boxes
	Mat_<Vec3b> screen(leftImg.rows, leftImg.cols);
	cvtColor(leftImg, screen, CV_GRAY2BGR);
		
	for(int i=0; i<(int)correspondences.size(); i++) {
		double scaledDisp = (double)correspondences[i].disparity() / maxDisp;
		Vec3b color;
		
		if(scaledDisp > 0.5)
			color = Vec3b(0, (1 - scaledDisp)*512, 255);
		else color = Vec3b(0, 255, scaledDisp*512);
			
		rectangle(screen, correspondences[i].imgLeft->pt - Point2f(2,2), correspondences[i].imgLeft->pt + Point2f(2, 2), 
		(Scalar) color, CV_FILLED);
		}

	// Display image and wait
	namedWindow("Stereo");
	imshow("Stereo", screen);
	waitKey();

	//cv::imshow("Image Left", I_l);
	//cv::imshow("Image Right", I_r);
	//cv::waitKey(0);

	delete leftFeatureDetector;
	delete rightFeatureDetector;

}
