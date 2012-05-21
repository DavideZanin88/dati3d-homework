
#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include <pcl/common/common_headers.h>

#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation
#include <pcl/features/pfh.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "my_datatype.h"
#include "cloud_io.h"


class myRegistration{

public:
	static PointCloudPtr registration(PointCloudPtr sr, PointCloudPtr target);


private:

	//Calcola le feature di una point cloud
	//cloud: la point cloud completa
	//filt: la point cloud con solo i keypoint
	static pcl::PointCloud<pcl::FPFHSignature33>::Ptr findFPFH(PointCloudPtr cloud, PointCloudPtr filt);

	//Trova i keypoint di una cloud
	static PointCloudPtr detectKeypoints(PointCloudPtr cloud);

};


#endif /* REGISTRATION_H_ */
