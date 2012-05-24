
#ifndef IO_H_
#define IO_H_

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

#include "my_datatype.h"
#include "extract_object.h"

class CloudIO{

public:

	/**
	 * Carica una point cloud da file ed effettua le seguenti operazioni
	 * - sposta il centroide a (0, 0, 0)
	 * - rimuove il tavolo
	 */
	static PointCloudPtr loadPointCloud(const std::string& path);


	/**
	 * Visualizza una point cloud
	 */
	static void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name);

	static void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, std::string name1,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2, std::string name2);

	static void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, std::string name1,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2, std::string name2,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3, std::string name3);

};


#endif /* IO_H_ */
