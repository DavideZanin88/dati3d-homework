
#include "mypointcloud.h"

using namespace std;
using namespace cv;
using namespace pcl;

MyPointCloud::MyPointCloud(Calibration& calib, Disparity& disp): calib(calib), disp(disp){
}


void MyPointCloud::visualize(const string& imgNameL, const string& imgNameR){

	Mat disparity, imgL, imgR;
//	imgL = imread(imgNameL.c_str(), CV_8UC1);
//	imgR = imread(imgNameR.c_str(), CV_8UC1);
	this->disp.computeDisparityImage(imgNameL, imgNameR, disparity);

	Mat disparityRoi(disparity, calib.getRoi());

	Mat colorImgTemp;
	Mat temp = imread("disparity/left.ppm");
	calib.rectfy(temp, colorImgTemp, LEFT_CAMERA);
	Mat colorImg(colorImgTemp, calib.getRoi());



	Mat recons3D;
	reprojectImageTo3D(disparityRoi, recons3D, calib.getQ(), true);
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

	const double max = 80;
	for (int r = 0; r < disparityRoi.rows; r++){
		for (int c = 0; c < disparityRoi.cols; c++){
			
			Vec3f xyzPoint = recons3D.at<Vec3f>(r, c);

			if (isnan(xyzPoint[0]) || isnan(xyzPoint[1]) || isnan(xyzPoint[2]))
				continue;
			if(fabs(xyzPoint[0]) > max || fabs(xyzPoint[1]) > max || fabs(xyzPoint[2]) > max)
				continue;

			PointXYZRGB cloudPoint;
			cloudPoint.x = xyzPoint[0];
			cloudPoint.y = xyzPoint[1];
			cloudPoint.z = xyzPoint[2];

			Vec3b rgbPoint = colorImg.at<Vec3b>(r, c);
			cloudPoint.b = rgbPoint[0]; //opencv usa la codifica bgr
			cloudPoint.g = rgbPoint[1];
			cloudPoint.r = rgbPoint[2];

			cloud->points.push_back (cloudPoint);
		}
	}

	cloud->width = (int) cloud->points.size();
	cloud->height = 1;

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	viewer.setBackgroundColor (0, 0, 0);
	viewer.addCoordinateSystem (0.1);
	viewer.addText ("HW1 cloud", 10, 10);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "HW1 cloud");

	std::cout << "Visualization... "<< std::endl;
	while (!viewer.wasStopped ())
	{
		viewer.spin ();
	}
	return;

}
