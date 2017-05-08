cmake_minimum_required(VERSION 2.8)
project( Hypertun-SR )
find_package( OpenCV REQUIRED)
find_package( PNG REQUIRED )
include_directories(${PNG_INCLUDE_DIR})
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
target_link_libraries( main.out sparsestereo ${OpenCV_LIBS} ${PNG_LIBRARY} )

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")

add_subdirectory(sparsestereo)



