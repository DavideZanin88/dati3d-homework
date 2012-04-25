
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

	Mat recons3D;
	reprojectImageTo3D(disparity, recons3D, calib.getQ(), true);

	//calcolac i valori max e min di z
	float minZ = +INFINITY, maxZ = -INFINITY;
	for (int r = 0; r < recons3D.rows; r++){
		for (int c = 0; c < recons3D.cols; c++){
			float z = fabs(recons3D.at<Vec3f>(r,c)[2]);
			if (z < minZ)
				minZ = z;
			if (z > maxZ && z < 10000)
				maxZ = z;
		}
	}
	float step = 255.0/fabs(maxZ-minZ);
	cout << minZ << " " << maxZ << " " << step << " " << endl;

	Mat new3D(recons3D.size(), CV_8UC3);
	range = new3D;
	for (int r = 0; r < recons3D.rows; r++){
		for (int c = 0; c < recons3D.cols; c++){
			float z = fabs(recons3D.at<Vec3f>(r,c)[2]);

			if (z < 10000){
				z = fabs(step*(z-minZ));
				range.at<Vec3b>(r,c)[0] = z;
				range.at<Vec3b>(r,c)[1] = z;
				range.at<Vec3b>(r,c)[2] = z;
			}else{
				range.at<Vec3b>(r,c)[0] = 0;
				range.at<Vec3b>(r,c)[1] = 0;
				range.at<Vec3b>(r,c)[2] = 0;
			}

		}
	}

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
