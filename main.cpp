#include "libs/triangulation/DelaunayTriangulation.h"
#include <iostream>
#include <vector>
//#include <stdio.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char const *argv[])
{
	cout << "Test" << endl;

	struct triangulateio in, out;
	in.numberofpoints = static_cast<int>(4);
	in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));

	float supportPts_array [8] = {0,0,1,1,0,1,1,0}; //{{1,2},{3,4},{5,6},{7,8}};
	//vector<int> m_vSupportPts(6,1);
	for (int i = 0; i < 8; ++i)
	{
		//m_vSupportPts[i] = i;
		cout << supportPts_array[i] << endl;
		in.pointlist[i] = supportPts_array[i];
	}

	/*for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 2; ++j)
			cout << m_vSupportPts[i][j] << endl;*/
	
	//in.numberofpoints = static_cast<int>(m_vSupportPts.size());
	//in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));

	//int k = 0;

	/*for (int32_t i = 0; i<m_vSupportPts.size(); i++) {
		in.pointlist[k++] = m_vSupportPts[i];
		in.pointlist[k++] = m_vSupportPts[i];
	}*/

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

	char parameters[] = "zQBn";
	triangulate(parameters, &in, &out, NULL);

	int k = 0;

	cout << "neighborlist" << endl;
	for (int i = 0; i < out.numberoftriangles; ++i)
	{
		cout << out.neighborlist[k] << endl;
		cout << out.neighborlist[k+1] << endl;
		cout << out.neighborlist[k+2] << endl;
		k += 3;
	}

	cout << "trianglelist" << endl;
	k = 0;
	for (int i = 0; i < out.numberoftriangles; ++i)
	{
		cout << out.trianglelist[k] << endl;
		cout << out.trianglelist[k+1] << endl;
		cout << out.trianglelist[k+2] << endl;
		k += 3;
	}
	//cout << out.pointlist[0] << endl;

	return 0;
}