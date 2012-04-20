
#include <iostream>
#include <list>
#include <string>

#include "cv.h"

#include "io.h"
#include "calibration.h"
#include "disparity.h"
#include "mypointcloud.h"


using namespace std;
using namespace cv;


int main(int argc, char **argv){

	Calibration calib("calibration_set");
	calib.stereoCalibration();

	Disparity disp(calib);

	MyPointCloud point(calib, disp);
	point.visualize("disparity/left.ppm", "disparity/right.ppm");




	cv::waitKey(0);
}




