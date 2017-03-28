#include <opencv2/opencv.hpp>
#include "../libs/triangulation/DelaunayTriangulation.h"
#include <iostream>
#include <cmath>

#ifndef DELAUNAY_TRIANGULATION_HPP
#define DELAUNAY_TRIANGULATION_HPP

void delaunay_triangulation(cv::Mat &S, int H, int W, cv::Mat &G, cv::Mat &T, cv::Mat &E);
#endif // DELAUNAY_TRIANGULATION_HPP