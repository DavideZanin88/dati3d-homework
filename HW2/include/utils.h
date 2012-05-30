
#ifndef UTILS_H_
#define UTILS_H_


#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/conditional_removal.h>

#include "my_datatype.h"


class Utils{

public:




	static PointCloudPtr voxel(const PointCloudPtr& cloud, float lx, float ly, float lz);
	static PointCloudPtr voxel(const PointCloudPtr& cloud, float l);


	static PointCloudPtr computeSIFT(const PointCloudPtr& cloud);


	static FeatureCloudPtr computeFPFH(const PointCloudPtr& cloud, const PointCloudPtr& filtered);


	static void setColor(const PointCloudPtr& cloud, char r, char g, char b);

	static void copyTo(const PointCloudPtr& src, const PointCloudPtr& dest);

	static PointCloudPtr conditionFilter(const PointCloudPtr& cloud, float minX, float maxX,
										 float minY, float maxY, float minZ, float maxZ);

};


#endif /* UTILS_H_ */
