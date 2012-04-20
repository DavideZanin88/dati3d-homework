
#ifndef DISPARITY_H_
#define DISPARITY_H_

#include <string>

#include <cv.h>
#include <highgui.h>

#include "calibration.h"


class Disparity{

	public:
		Disparity(Calibration& calib);
		void computeDisparityImage(const std::string& imgNameL, const std::string& imgNameR, cv::Mat& disparity);
		void computeRangeImage(const std::string& imgNameL, const std::string& imgNameR, cv::Mat& range);

	private:
		void initSGBM(cv::Mat img);
		void initBM(cv::Mat img);

	private:
		Calibration& 	calib;
		cv::StereoSGBM 	sgbm;

};


#endif /* DISPARITY_H_ */
