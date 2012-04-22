
#include "disparity.h"


using namespace cv;
using namespace std;


Disparity::Disparity(Calibration& calib):calib(calib){
}

void Disparity::computeDisparityImage(const string& imgNameL, const string& imgNameR, Mat& disparity){
	Mat imgL = imread(imgNameL, CV_8UC1);
	Mat imgR = imread(imgNameR, CV_8UC1);

	Mat rectL, rectR;
	this->calib.rectfy(imgL, rectL, LEFT_CAMERA);
	this->calib.rectfy(imgR, rectR, RIGHT_CAMERA);

	initSGBM(imgL);

	sgbm(rectL, rectR, disparity);
}

void Disparity::computeRangeImage(const string& imgNameL, const string& imgNameR, Mat& range){
	Mat disparity;
	computeDisparityImage(imgNameL, imgNameR, disparity);
	disparity.convertTo(range, CV_8U, 255/((sgbm.numberOfDisparities)*16.));
}


void Disparity::initSGBM(cv::Mat img){
	int numberOfDisparities = ((img.size().width/8) + 15) & -16; // default width/8
	sgbm.preFilterCap = 4;
	sgbm.SADWindowSize =  3;
	int cn = img.channels();
	sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 16;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 5;
	sgbm.speckleWindowSize = 250;
	sgbm.speckleRange = 48;
	sgbm.disp12MaxDiff = -1;
	sgbm.fullDP = 1;
}
