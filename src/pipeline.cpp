#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


void pipeline(){

	cv::Mat imgL = cv::imread("../data/data_scene_flow/testing/image_2/000000_10.png");
	cv::Mat imgR = cv::imread("../data/data_scene_flow/testing/image_3/000000_10.png");
	cv::imshow("Image Left", imgL);
	cv::imshow("Image Right", imgR);
	cv::waitKey(0);
	std::cout << "Test" << std::endl;

}