#include "delaunay_triangulation.hpp"
#include "../libs/triangulation/DelaunayTriangulation.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>

using namespace std;

float sign (cv::Point3f p1, cv::Point3f p2, cv::Point3f p3)
{
    return (p1.y - p3.y) * (p2.x - p3.x) - (p2.y - p3.y) * (p1.x - p3.x);
}
bool PointInTriangle (cv::Point3f pt, cv::Point3f v1, cv::Point3f v2, cv::Point3f v3)
{
    bool b1, b2, b3;

    b1 = sign(pt, v1, v2) < 0.0f;
    b2 = sign(pt, v2, v3) < 0.0f;
    b3 = sign(pt, v3, v1) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}


void delaunay_triangulation(cv::Mat &S, cv::Mat &G, cv::Mat I_l){

	std::cout << "delaunay_triangulation.cpp" << std::endl;

	//cv::Mat surfaceNormal;
	//cv::Mat containedPixels;


	// Store support points into input variable
	int N = S.cols;

	struct triangulateio in, out;
	in.numberofpoints = static_cast<int>(N);
	in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));

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
	//cout << "edgelist: " << out.numberofedges << endl;
	k = 0;
	for (int i = 0; i < out.numberofedges; ++i) {
		//cout << out.edgelist[k] << ", ";
		//cout << out.edgelist[k+1] << endl;
		G.push_back(out.edgelist[k]);
		G.push_back(out.edgelist[k+1]);
		k += 2;
	}

	// Compute normal vector of each triangle surface
	cv::Point3f pts[N];
	//float surface_normal[N][3];
	int dummy_array[N];
	cv::Point3f n_plane[N];
	

	for (int i = 0; i < N; ++i) {
		pts[i].x = S.at<float>(0,i);
		pts[i].y = S.at<float>(1,i);
		pts[i].z = S.at<float>(2,i);

	}
	cout << "normal vectors" << endl;
	k = 0;
	for (int i = 0; i < out.numberoftriangles; ++i) {
		cv::Point3f a, b;

		a = pts[out.trianglelist[k]] - pts[out.trianglelist[k+1]];
		b = pts[out.trianglelist[k]] - pts[out.trianglelist[k+2]];
		n_plane[i] = a.cross(b);

		float norm = sqrt(n_plane[i].x * n_plane[i].x + 
						  n_plane[i].y * n_plane[i].y +
						  n_plane[i].z * n_plane[i].z);
		n_plane[i].x /= norm;
		n_plane[i].y /= norm;
		n_plane[i].z /= norm;

		//surface_normal[i][0] = surface_normal[i][0]/norm;
		//surface_normal[i][1] = surface_normal[i][1]/norm;
		//surface_normal[i][2] = surface_normal[i][2]/norm;
		cout << a << ", " << b << "-> " << n_plane[i] << endl;
		k += 3;
	}

	// Assign each pixel to the corresponding triangle
	cv::Mat triangleLabel = cv::Mat(I_l.rows, I_l.cols, CV_8U, 0.0);
		
	for (int y = 0; y < I_l.rows; ++y) {
		for (int x = 0; x < I_l.cols; ++x) {
			k = 0;
			for (int i = 0; i < out.numberoftriangles; ++i) {
				cv::Point3f pt;
				pt.x = y;
				pt.y = x;
				cv::Point3f v1 = pts[out.trianglelist[k]];
				cv::Point3f v2 = pts[out.trianglelist[k+1]];
				cv::Point3f v3 = pts[out.trianglelist[k+2]];
				if (PointInTriangle(pt, v1, v2, v3)) {
					I_l.at<int>(x,y,0) = i*20;
				}
				k += 3;
			}
		}
	}
	cout << "rows: " << I_l.rows << endl;
	cout << "cols: " << I_l.cols << endl;

	
	cv::imshow("Image label", I_l);
	cv::waitKey(0);

	free(in.pointlist);
	free(out.pointlist);
	free(out.trianglelist);

}