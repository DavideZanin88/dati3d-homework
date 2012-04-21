
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

	//prova la rettifica
	Mat imgL = imread("calibration_set/left117.ppm");
	Mat rectfyL;
	calib.rectfy(imgL, rectfyL, LEFT_CAMERA);
	imwrite("result/rectfyLeft117.ppm", rectfyL);

	Mat imgR = imread("calibration_set/right117.ppm");
	Mat rectfyR;
	calib.rectfy(imgL, rectfyR, RIGHT_CAMERA);
	imwrite("result/rectfyRight117.ppm", rectfyR);
	//------

	//prova disparity map
	Mat disparity;
	disp.computeDisparityImage("disparity/left.ppm", "disparity/right.ppm", disparity);
	imwrite("result/disparity.ppm", disparity);
	//-----

	//prova range immage
	Mat range;
	disp.computeRangeImage("disparity/left.ppm", "disparity/right.ppm", range);
	imwrite("result/range.ppm", range);
	//---


	MyPointCloud point(calib, disp);
	point.visualize("disparity/left.ppm", "disparity/right.ppm");




	cv::waitKey(0);
}




