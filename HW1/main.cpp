
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

	cout << "Calibrazione delle fotocamere..." << endl;
	Calibration calib("calibration_set");
	calib.calibration();

	cout << "Calcolo e visualizzazione della point cloud..." << endl;
	Disparity disp(calib);

	Mat imgL = imread("calibration_set/left117.ppm");
	Mat imgR = imread("calibration_set/right117.ppm");
	Mat result;

	//prova undistort
	calib.undistort(imgL, result, LEFT_CAMERA);
	imwrite("result/undistortLeft117.ppm", result);

	calib.undistort(imgR, result, RIGHT_CAMERA);
	imwrite("result/undistortRight117.ppm", result);
	//-----

	//prova la rettifica
	calib.rectfy(imgL, result, LEFT_CAMERA);
	imwrite("result/rectfyLeft117.ppm", result);

	calib.rectfy(imgL, result, RIGHT_CAMERA);
	imwrite("result/rectfyRight117.ppm", result);
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




