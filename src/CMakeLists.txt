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
				support_resampling.cpp)
target_link_libraries( main.out ${OpenCV_LIBS} )

set(CMAKE_CXX_FLAGS "-O3 -DNDEBUG -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Find Required libraries
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

# Setting include directories
include_directories(${PROJECT_SOURCE_DIR}/src)



