
#ifndef IO_H_
#define IO_H_


#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>

#include "my_datatype.h"


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
	static void visualize(const PointCloudPtr& cloud, const std::string& name);

	static void visualize(const PointCloudPtr& cloud1, const std::string& name1,
						  const PointCloudPtr& cloud2, const std::string& name2);

	static void visualize(const PointCloudPtr& cloud1, const std::string& name1,
						  const PointCloudPtr& cloud2, const std::string& name2,
						  const PointCloudPtr& cloud3, const std::string& name3);

};


#endif /* IO_H_ */
