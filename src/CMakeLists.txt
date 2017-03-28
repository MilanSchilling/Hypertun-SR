cmake_minimum_required(VERSION 2.8)
project( Hypertun-SR )
find_package( OpenCV REQUIRED )
add_executable( main.out
				main.cpp 
				pipeline.cpp
				sparse_stereo.cpp
				delaunay_triangulation.cpp
				disparity_interpolation.cpp
				cost_evaluation.cpp
				disparity_refinement.cpp
				support_resampling.cpp
				../libs/triangulation/DelaunayTriangulation.cpp)
target_link_libraries( main.out ${OpenCV_LIBS} )

