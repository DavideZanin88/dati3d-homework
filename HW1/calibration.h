
#ifndef CALIBRATION_H_
#define CALIBRATION_H_


#include <list>
#include <string>

#include "cv.h"
#include "highgui.h"

#include "io.h"


class Calibration{

	public:
		Calibration(std::string photoDir);

		void stereoCalibration();
		void calibrateCamera();
		void calibrationStereoCamera();
		void rectfy(const cv::Mat& img, cv::Mat& rectfy, CameraType type);

		cv::Mat getQ();

	private:
		std::string photoDir;
		cv::Mat 	M[2], D[2];
		cv::Mat 	RR, T, E, F;
		cv::Mat 	R[2], P[2], Q;

		std::vector<std::vector<cv::Point2f> > imagePoints[2];
		std::vector<cv::Point3f> objectCoord;
		cv::Mat m[2][2];

		PhotoIO io;

	public:
		static const int patterSizeX = 7;
		static const int patterSizeY = 5;

};


#endif /* CALIBRATION_H_ */
