#ifndef SUPPORT_RESAMPLING_HPP
#define SUPPORT_RESAMPLING_HPP

#include <opencv2/opencv.hpp>

void support_resampling(cv::Mat &C_g, cv::Mat &C_b, 
                            cv::Mat &S_it, cv::Mat &S_it_next);
#endif // SUPPORT_RESAMPLING_HPP