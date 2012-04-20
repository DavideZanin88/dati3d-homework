
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
		void calibrateCamera(std::list<std::string> l, cv::Mat& intrinsic,
				cv::Mat& distCoeffs, std::vector<std::vector<cv::Point2f> >& imagePoints);
		void calibrationStereoCamera();
		void rectfy(const cv::Mat& img, cv::Mat& rectfy, CameraType type);

		cv::Mat getQ();
		cv::Rect getRoi();

	private:
		std::string photoDir;
		cv::Mat 	M[2], D[2];
		cv::Mat 	RR, T, E, F;
		cv::Mat 	R[2], P[2], Q;
		cv::Rect 	roi[2];

		std::vector<std::vector<cv::Point2f> > imagePoints[2];
		std::vector<cv::Point3f> objectCoord;
		cv::Mat m[2][2];

	public:
		static const int patterSizeX = 7;
		static const int patterSizeY = 5;

};


#endif /* CALIBRATION_H_ */
