
#include "mypointcloud.h"

using namespace std;
using namespace cv;
using namespace pcl;

MyPointCloud::MyPointCloud(Calibration& calib, Disparity& disp): calib(calib), disp(disp){
}


void MyPointCloud::visualize(const string& imgNameL, const string& imgNameR){

	Mat disparity;
	this->disp.computeDisparityImage(imgNameL, imgNameR, disparity);

	Mat colorImg;
	Mat temp = imread("disparity/left.ppm");
	calib.rectfy(temp, colorImg, LEFT_CAMERA);

	Mat recons3D;
	reprojectImageTo3D(disparity, recons3D, calib.getQ(), true);
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

	const double max = 80;
	for (int r = 0; r < disparity.rows; r++){
		for (int c = 0; c < disparity.cols; c++){

			Vec3f xyzPoint = recons3D.at<Vec3f>(r, c);
			if(fabs(xyzPoint[2]) > max){
				continue;
			}

			PointXYZRGB cloudPoint;
			cloudPoint.x = xyzPoint[0];
			cloudPoint.y = xyzPoint[1];
			cloudPoint.z = xyzPoint[2];

			Vec3b bgrPoint = colorImg.at<Vec3b>(r, c);
			cloudPoint.b = bgrPoint[0]; //opencv usa la codifica bgr
			cloudPoint.g = bgrPoint[1];
			cloudPoint.r = bgrPoint[2];

			cloud->points.push_back (cloudPoint);
		}
	}

	cloud->width = (int) cloud->points.size();
	cloud->height = 1;

	visualization::PCLVisualizer viewer("PCL Viewer");

	viewer.setBackgroundColor (0, 0, 0);
	viewer.addCoordinateSystem (0.1);
	viewer.addText ("HW1 cloud", 10, 10);
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer.addPointCloud<PointXYZRGB> (cloud, rgb, "HW1 cloud");

	std::cout << "Visualization... "<< std::endl;
	while (!viewer.wasStopped ())
	{
		viewer.spin ();
	}
	return;

}
