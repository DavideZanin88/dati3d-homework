
#ifndef MYPOINTCLOUD_H_
#define MYPOINTCLOUD_H_

#include <cv.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include "calibration.h"
#include "disparity.h"


class MyPointCloud{

	public:
		MyPointCloud(Calibration& calib, Disparity& disp);

		void visualize(const std::string& imgNameL, const std::string& imgNameR);

	private:
		Calibration& 	calib;
		Disparity&		disp;

};



#endif /* MYPOINTCLOUD_H_ */
