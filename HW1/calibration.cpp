
#include "calibration.h"


using namespace cv;
using namespace std;


Calibration::Calibration(string photoDir){
	this->photoDir = photoDir;

	for(int j = 0; j < patterSizeX*patterSizeY; j++)
		objectCoord.push_back(Point3f(j/patterSizeX*8, j%patterSizeX*8, 0.0f));

}

void Calibration::stereoCalibration(){

	cout << "Calibro la fotocamera sinistra" << endl;
	list<string> list = getFilenameList(this->photoDir, LEFT_CAMERA);
	calibrateCamera(list, this->M[0], this->D[0], this->imagePoints[0]);

	cout << "Calibro la fotocamera destra" << endl;
	list = getFilenameList(this->photoDir, RIGHT_CAMERA);
	calibrateCamera(list, this->M[1], this->D[1], this->imagePoints[1]);

	cout << "Calibrazione stereo" << endl;
	calibrationStereoCamera();

}

void Calibration::calibrateCamera(list<string> l, Mat& intrinsic, Mat &distCoeffs,
		vector<vector<Point2f> > &imagePoints){

	Size patternSize(patterSizeX, patterSizeY);

	vector<vector<Point3f> > objectPoints; //vettore delle coordinate dei pti nel mondo

	Mat img;
	list<string>::iterator it;
	for ( it = l.begin(); it != l.end(); it++){ //scorre tutta la lista di immagini
		cout << *it << endl; //TODO debug

		img = imread(this->photoDir+"/"+*it, 0);

		vector<Point2f> corners;
		if (!findChessboardCorners(img, patternSize, corners,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE |
				CV_CALIB_CB_FILTER_QUADS | CALIB_CB_FAST_CHECK )){
			cout << "Corner non trovati: "+*it << endl;
			continue;
		}

		img = imread(this->photoDir+"/"+*it, CV_8UC1);
		cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
				TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

		imagePoints.push_back(corners);
		objectPoints.push_back(objectCoord);
	}

	intrinsic = Mat(3, 3, CV_32FC1);
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	cv::calibrateCamera(objectPoints, imagePoints, img.size(), intrinsic, distCoeffs, rvecs, tvecs);

}



void Calibration::calibrationStereoCamera(){

	if (imagePoints[0].size() != imagePoints[1].size()){
		cout << "calibrationStereoCamera: errore imagePointR/L hanno dimensioni divesere!" << endl;
		cout << "L size: " << imagePoints[0].size() << " R size: " << imagePoints[1].size() << endl;
		return;
	}

	vector<vector<Point3f> > objectPoints; //vettore delle coordinate dei pti nel mondo

	vector<vector<Point2f> >::iterator it;
	for ( it = imagePoints[0].begin(); it != imagePoints[0].end(); it++){ //scorre tutta la lista di immagini
		objectPoints.push_back(objectCoord);
	}

	Mat img = imread("disparity/left.ppm", CV_8UC1);

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], M[0], D[0], M[1], D[1],
			img.size(), RR, T, E, F,
			cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), CV_CALIB_FIX_INTRINSIC);

	cout << "done with RMS error=" << rms << endl;


	stereoRectify(M[0], D[0], M[1], D[1], img.size(), RR, T, R[0], R[1], P[0], P[1], Q,
			CALIB_ZERO_DISPARITY, 0, img.size(), &roi[0], &roi[1]);

	initUndistortRectifyMap(M[0], D[0], R[0], P[0], img.size(), CV_16SC2, m[0][0], m[0][1]);
	initUndistortRectifyMap(M[1], D[1], R[1], P[1], img.size(), CV_16SC2, m[1][0], m[1][1]);

}

void Calibration::rectfy(const cv::Mat& img, cv::Mat& rectfy, CameraType type){
	int camera;
	if (type == LEFT_CAMERA)
		camera = 0;
	else
		camera = 1;

	remap(img, rectfy, m[camera][0], m[camera][1], INTER_LINEAR);
}

Mat Calibration::getQ(){
	return this->Q;
}

Rect Calibration::getRoi(){
	return this->roi[0] & this->roi[1];
}
