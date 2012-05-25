
#ifndef EXTRACT_OBJECT_H_
#define EXTRACT_OBJECT_H_


#include <pcl/common/common_headers.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "my_datatype.h"
#include "cloud_io.h"
#include "utils.h"


class ExtractObject{

public:
	static PointCloudPtr extractPioli(PointCloudPtr cloud);
	static PointCloudPtr extractPioli2(PointCloudPtr cloud);

	static PointCloudPtr extractCable(PointCloudPtr cloud);

	static bool isCableCorrect(PointCloudPtr pioli, PointCloudPtr cable);

	static PointCloudPtr segmentation(PointCloudPtr cloud, double threshold);


private:


	static std::vector<pcl::PointIndices> clusterization(PointCloudPtr cloud, double tolerance, double min, double max);

	static bool isCable(PointCloudPtr cluster);

	static bool isPioloSx(PointCloudPtr cloud);
	static bool isPioloDx(PointCloudPtr cloud);


public:

	static const float SX_MIN_X = -27;
	static const float SX_MIN_Y = -44;
	static const float SX_MIN_Z = -9.5;

	static const float SX_MAX_X = -22;
	static const float SX_MAX_Y = -37.5;
	static const float SX_MAX_Z = -7.5;


	static const float DX_MIN_X = -22.5;
	static const float DX_MIN_Y = -44;
	static const float DX_MIN_Z = -8.5;

	static const float DX_MAX_X = -17.5;
	static const float DX_MAX_Y = -38;
	static const float DX_MAX_Z = -6.5;


	static const float CABLE_MIN_X = -30;
	static const float CABLE_MIN_Y = -55;
	static const float CABLE_MIN_Z = -15;

	static const float CABLE_MAX_X = -5;
	static const float CABLE_MAX_Y = -35;
	static const float CABLE_MAX_Z = -2;

};


#endif /* EXTRACT_OBJECT_H_ */
