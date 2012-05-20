
#ifndef EXTRACT_OBJECT_H_
#define EXTRACT_OBJECT_H_


#include <pcl/common/common_headers.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "my_datatype.h"
#include "cloud_io.h"


class ExtractObject{

public:
	static PointCloudPtr extractPioli(PointCloudPtr cloud);

	static PointCloudPtr extractCable(PointCloudPtr cloud);

	static bool isCableCorrect(PointCloudPtr pioli, PointCloudPtr cable);


private:
	static PointCloudPtr segmentation(PointCloudPtr cloud, double threshold);

	static std::vector<pcl::PointIndices> clusterization(PointCloudPtr cloud, double tolerance, double min, double max);

	static bool isCable(PointCloudPtr cluster);

};


#endif /* EXTRACT_OBJECT_H_ */
