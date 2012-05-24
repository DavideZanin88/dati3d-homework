
#ifndef UTILS_H_
#define UTILS_H_


#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>

#include "my_datatype.h"


class Utils{

public:




	static PointCloudPtr voxel(PointCloudPtr cloud, float lx, float ly, float lz);
	static PointCloudPtr voxel(PointCloudPtr cloud, float l);


	static PointCloudPtr computeSIFT(PointCloudPtr cloud);


	static FeatureCloudPtr computeFPFH(PointCloudPtr cloud, PointCloudPtr filtered);

};


#endif /* UTILS_H_ */
