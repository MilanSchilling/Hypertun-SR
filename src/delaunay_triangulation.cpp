#include "delaunay_triangulation.hpp"
#include "../libs/triangulation/DelaunayTriangulation.h"
#include <iostream>
#include <stdlib.h>
#include <opencv2/core/core.hpp>

using namespace std;

void delaunay_triangulation(cv::Mat &S, cv::Mat &G){

	std::cout << "delaunay_triangulation.cpp" << std::endl;

	int N = S.cols;

	struct triangulateio in, out;
	in.numberofpoints = static_cast<int>(N);
	in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));

	float supportPts_array [8] = {0,0,1,1,0,1,1,0};
	int k = 0;
	for (int i = 0; i < N; ++i) {
		cout << S.at<float>(0,i) << ", " << S.at<float>(1,i) << endl;
		in.pointlist[k++] = S.at<float>(0,i);
		in.pointlist[k++] = S.at<float>(1,i);
	}

	in.numberofpointattributes = 0;
	in.pointattributelist = NULL;
	in.pointmarkerlist = NULL;
	in.numberofsegments = 0;
	in.numberofholes = 0;
	in.numberofregions = 0;
	in.regionlist = NULL;

	// outputs
	out.pointlist = NULL;
	out.pointattributelist = NULL;
	out.pointmarkerlist = NULL;
	out.trianglelist = NULL;
	out.triangleattributelist = NULL;
	out.neighborlist = NULL;
	out.segmentlist = NULL;
	out.segmentmarkerlist = NULL;
	out.edgelist = NULL;
	out.edgemarkerlist = NULL;

	char parameters[] = "zQBne";
	triangulate(parameters, &in, &out, NULL);

	cout << "trianglelist: " << out.numberoftriangles << endl;
	k = 0;
	for (int i = 0; i < out.numberoftriangles; ++i) {
		cout << out.trianglelist[k] << ", ";
		cout << out.trianglelist[k+1] << ", ";
		cout << out.trianglelist[k+2] << endl;
		k += 3;
	}
	cout << "edgelist: " << out.numberofedges << endl;
	k = 0;
	for (int i = 0; i < out.numberofedges; ++i) {
		cout << out.edgelist[k] << ", ";
		cout << out.edgelist[k+1] << endl;
		G.push_back(out.edgelist[k]);
		G.push_back(out.edgelist[k+1]);
		k += 2;
	}


}