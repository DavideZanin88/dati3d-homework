
#ifndef MY_DATATYPE_H_
#define MY_DATATYPE_H_


#include <pcl/common/common_headers.h>


typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr 					PointCloudPtr;
//typedef pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> 	KeypointCloudPtr;
typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr				FeatureCloudPtr;


#endif /* DATATYPE_H_ */
