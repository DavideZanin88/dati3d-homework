
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

	Mat imgL = imread("disparity/left.ppm");
	Mat imgR = imread("disparity/right.ppm");
	Mat result;

	//prova undistort
	calib.undistort(imgL, result, LEFT_CAMERA);
	imwrite("result/undistortLeft.jpg", result);

	calib.undistort(imgR, result, RIGHT_CAMERA);
	imwrite("result/undistortRight.jpg", result);
	//-----

	//prova la rettifica
	calib.rectfy(imgL, result, LEFT_CAMERA);
	imwrite("result/rectfyLeft.jpg", result);

	calib.rectfy(imgL, result, RIGHT_CAMERA);
	imwrite("result/rectfyRight.jpg", result);
	//------

	//prova disparity map
	Mat disparity;
	disp.computeDisparityImage("disparity/left.ppm", "disparity/right.ppm", disparity);
	Mat disparityNorm;
	normalize(disparity, disparityNorm, 0, 256, CV_MINMAX, CV_8UC1);
	imwrite("result/disparity.jpg", disparityNorm);
	//-----

	//prova range immage
	Mat range;
	disp.computeRangeImage("disparity/left.ppm", "disparity/right.ppm", range);
	imwrite("result/range.jpg", range);
	//---

	MyPointCloud point(calib, disp);
	point.visualize("disparity/left.ppm", "disparity/right.ppm");
}




