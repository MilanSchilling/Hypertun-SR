#ifndef SPARSE_STEREO_HPP
#define SPARSE_STEREO_HPP

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
#include "../../exFAST_SparseStereo/src/sparsestereo/exception.h"
#include "../../exFAST_SparseStereo/src/sparsestereo/extendedfast.h"
#include "../../exFAST_SparseStereo/src/sparsestereo/stereorectification.h"
#include "../../exFAST_SparseStereo/src/sparsestereo/sparsestereo-inl.h"
#include "../../exFAST_SparseStereo/src/sparsestereo/census-inl.h"
#include "../../exFAST_SparseStereo/src/sparsestereo/imageconversion.h"
#include "../../exFAST_SparseStereo/src/sparsestereo/censuswindow.h"

void sparse_stereo(cv::Mat I_l, cv::Mat I_r);

#endif // SPARSE_STEREO_HPP
