

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main() {

	Mat image = imread("../../data/kitti/00/image_0/000000.png");
	imshow("Image", image);
	waitKey(0);

}