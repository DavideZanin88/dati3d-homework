
#include "mypointcloud.h"

using namespace std;
using namespace pcl;
using namespace registration;


PointCloudPtr MyPointCloud::pclRegister(PointCloudPtr src, PointCloudPtr target){

	PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
	VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud (src);
	sor.setLeafSize (2.0f, 2.0f, 2.0f);
	sor.filter (*cloud_filtered);
	src = cloud_filtered;

	PointCloud<PointXYZRGB>::Ptr cloud_filtered2(new PointCloud<PointXYZRGB>);
	sor.setInputCloud (target);
	sor.setLeafSize (2.0f, 2.0f, 2.0f);
	sor.filter (*cloud_filtered2);
	target = cloud_filtered2;

	cout << "# point: " << src->size() << ", " << target->size() << endl;
	CloudIO::visualize(src, "src", target, "target");

	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(src);
	icp.setInputTarget(target);

	icp.setMaximumIterations(1);
	icp.setMaxCorrespondenceDistance(50);
	icp.setRANSACOutlierRejectionThreshold(0.05);


	PointCloudPtr aligned(new PointCloud<PointXYZRGB>);
	icp.align (*aligned);

	icp.getFinalTransformation();

	for (int i = 0; i < src->points.size(); i++){
		src->points[i].r = 0;
		src->points[i].g = 0;
		src->points[i].b = 255;
	}

	for (int i = 0; i < target->points.size(); i++){
		target->points[i].r = 255;
		target->points[i].g = 0;
		target->points[i].b = 0;
	}

	for (int i = 0; i < aligned->points.size(); i++){
			aligned->points[i].r = 0;
			aligned->points[i].g = 255;
			aligned->points[i].b = 0;
		}
	CloudIO::visualize(src, "src", target, "target", aligned, "aligned");
	return aligned;

}



