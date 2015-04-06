
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

		void calibration();
		void calibrateCamera();
		void calibrationStereoCamera();
		bool findCorner(const cv::Mat& img, std::vector<cv::Point2f>& corners);
		void rectfy(const cv::Mat& img, cv::Mat& rectfy, CameraType type);
		void undistort(const cv::Mat& img, cv::Mat& undistort, CameraType type);

		cv::Mat getQ();

	private:
		cv::Mat 	M[2], D[2];
		cv::Mat 	RR, T;
		cv::Mat 	R[2], P[2], Q;
		cv::Mat 	m[2][2];

		std::vector<std::vector<cv::Point2f> > imagePoints[2];
		cv::Size	patternSize;
		std::vector<cv::Point3f> objectCoord;

		std::string photoDir;
		PhotoIO io;

	public:
		static const int patterSizeX = 7;
		static const int patterSizeY = 5;

};


#endif /* CALIBRATION_H_ */
