
#include "mypointcloud.h"

using namespace std;
using namespace pcl;
using namespace registration;


PointCloudPtr MyPointCloud::pclRegister(PointCloudPtr src, PointCloudPtr target){

//	PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
//	VoxelGrid<PointXYZRGB> sor;
//	sor.setInputCloud (src);
//	sor.setLeafSize (0.5f, 0.5f, 0.5f);
//	sor.filter (*cloud_filtered);
//	src = cloud_filtered;
//
//	sor.setInputCloud (target);
//	sor.setLeafSize (0.5f, 0.5f, 0.5f);
//	sor.filter (*cloud_filtered);
//	target = cloud_filtered;

	cout << "# point: " << src->size() << ", " << target->size() << endl;

	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(src);
	icp.setInputTarget(target);

	icp.setMaximumIterations(1);
	icp.setMaxCorrespondenceDistance(10);
	icp.setRANSACOutlierRejectionThreshold(0.05);

	PointCloudPtr aligned(new PointCloud<PointXYZRGB>);
	icp.align (*aligned);

	icp.getFinalTransformation();

	return aligned;

}



