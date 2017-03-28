#ifndef DISPARITY_REFINEMENT_HPP
#define DISPARITY_REFINEMENT_HPP

#include <opencv2/opencv.hpp>

void disparity_refinement(cv::Mat &D_it, cv::Mat &C_it, cv::Mat &C_f);

#endif // DISPARITY_REFINEMENT_HPP