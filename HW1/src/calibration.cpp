
#include "calibration.h"


using namespace cv;
using namespace std;


Calibration::Calibration(string photoDir){
	this->photoDir = photoDir;
	this->patternSize = Size(patterSizeX, patterSizeY);

	for(int j = 0; j < patterSizeX*patterSizeY; j++)
		objectCoord.push_back(Point3f(j/patterSizeX*8, j%patterSizeX*8, 0.0f));
}

void Calibration::calibration(){

	calibrateCamera();
	calibrationStereoCamera();

	//salva i valori delle matrici su file
	writeMatFile("value/M0.yml", "M0", this->M[0]);
	writeMatFile("value/M1.yml", "M1", this->M[1]);

	writeMatFile("value/D0.yml", "D0", this->D[0]);
	writeMatFile("value/D1.yml", "D1", this->D[1]);

	writeMatFile("value/RR.yml", "RR", this->RR);
	writeMatFile("value/T.yml", "T", this->T);
//	writeMatFile("value/E.yml", "E", this->E);
//	writeMatFile("value/F.yml", "F", this->F);

	writeMatFile("value/R0.yml", "R0", this->R[0]);
	writeMatFile("value/R1.yml", "R1", this->R[1]);

	writeMatFile("value/P0.yml", "P0", this->P[0]);
	writeMatFile("value/P1.yml", "P1", this->P[1]);

	writeMatFile("value/Q.yml", "Q", this->Q);

//	writeMatFile("value/mx0.yml", "mx0", this->m[0][0]);
//	writeMatFile("value/my0.yml", "my0", this->m[0][1]);
//	writeMatFile("value/mx1.yml", "mx1", this->m[1][0]);
//	writeMatFile("value/my1.yml", "my1", this->m[1][1]);

}

void Calibration::calibrateCamera(){
	vector<vector<Point3f> > objectPoints; //vettore delle coordinate dei pti nel mondo

	Mat imgL, imgR;
	while (!this->io.isEmpty()){
		io.getNext(imgL, imgR);
		vector<Point2f> corners[2];

		if (findCorner(imgL, corners[0]) && findCorner(imgR, corners[1])){
			this->imagePoints[0].push_back(corners[0]);
			this->imagePoints[1].push_back(corners[1]);
			objectPoints.push_back(objectCoord);
		}
	}

	M[0] = Mat(3, 3, CV_32FC1);
	M[1] = Mat(3, 3, CV_32FC1);
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	double rms = cv::calibrateCamera(objectPoints, imagePoints[0], imgL.size(), M[0], D[0], rvecs, tvecs);
	cout << "Re-projection error calibrazione fotocamera sx: " << rms << endl;
	rms = cv::calibrateCamera(objectPoints, imagePoints[1], imgL.size(), M[1], D[1], rvecs, tvecs);
	cout << "Re-projection error calibrazione fotocamera dx: " << rms << endl;
}


bool Calibration::findCorner(const Mat& img, vector<Point2f>& corners){
	if (!findChessboardCorners(img, this->patternSize, corners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE)){
		cout << "Corner non trovati " << endl;
		return false;
	}

	cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
			TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

	return true;
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

	Mat E, F;
	Mat img = imread("disparity/left.ppm", CV_8UC1);
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], M[0],
			D[0], M[1], D[1], img.size(), RR, T, E, F,
			cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), CV_CALIB_FIX_INTRINSIC);

	cout << "Re-projection error calibrazione stereo: " << rms << endl;


	stereoRectify(M[0], D[0], M[1], D[1], img.size(), RR, T, R[0], R[1], P[0], P[1], Q,
			CALIB_ZERO_DISPARITY, 0, img.size());

	initUndistortRectifyMap(M[0], D[0], R[0], P[0], img.size(), CV_16SC2, m[0][0], m[0][1]);
	initUndistortRectifyMap(M[1], D[1], R[1], P[1], img.size(), CV_16SC2, m[1][0], m[1][1]);

}

void Calibration::rectfy(const cv::Mat& img, cv::Mat& rectfy, CameraType type){
	int camera;
	type == LEFT_CAMERA ? camera = 0: camera = 1;
	remap(img, rectfy, m[camera][0], m[camera][1], INTER_LINEAR);
}

void Calibration::undistort(const Mat& img, Mat& undistort, CameraType type){
	int camera;
	type == LEFT_CAMERA ? camera = 0: camera = 1;
	cv::undistort(img, undistort, M[camera], D[camera]);
}

Mat Calibration::getQ(){
	return this->Q;
}
