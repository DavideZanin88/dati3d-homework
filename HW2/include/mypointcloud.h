
#ifndef MYPOINTCLOUD_H_
#define MYPOINTCLOUD_H_


#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>	// for computing normals with multi-core implementation
#include <pcl/features/fpfh_omp.h>		// for computing FPFH with multi-core implementation

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include "cloud_io.h"
#include "extract_object.h"
#include "my_datatype.h"


class MyPointCloud{

public:

	//registra la cloud source nella cloud target
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		pclRegister(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr target);

};


#endif /* MYPOINTCLOUD_H_ */
