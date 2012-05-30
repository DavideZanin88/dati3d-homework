
#ifndef EXTRACT_OBJECT_H_
#define EXTRACT_OBJECT_H_


#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "my_datatype.h"
#include "cloud_io.h"
#include "utils.h"


class ExtractObject{


public:
	/** Estrae i pioli dalla point cloud **/
	static PointCloudPtr extractPioli(const PointCloudPtr& cloud);

	/** Estrae il cavo dalla point cloud **/
	static PointCloudPtr extractCable(const PointCloudPtr& cloud);

	/** Date la point cloud dei pioli e quella del cavo ritorna true se il cavo è
	 *  nella posizione corretta **/
	static bool isCableCorrect(const PointCloudPtr& pioli, const PointCloudPtr& cable);


private:

	/** Esegue la clousterizzazione **/
	static std::vector<pcl::PointIndices> clusterization(const PointCloudPtr& cloud, double tolerance,
															double min, double max);

	/** Ritorna true se la point cloud è parte del cavo **/
	static bool isCable(const PointCloudPtr& cluster);

	/** Ritorna true se la point cloud è un piolo **/
	static bool isPioloSx(const PointCloudPtr& cloud);
	static bool isPioloDx(const PointCloudPtr& cloud);


public:

	static const float SX_MIN_X = -27.5;
	static const float SX_MIN_Y = -43.5;
	static const float SX_MIN_Z = -9.5;

	static const float SX_MAX_X = -21.5;
	static const float SX_MAX_Y = -37;
	static const float SX_MAX_Z = -7.5;


	static const float DX_MIN_X = -22;
	static const float DX_MIN_Y = -43.5;
	static const float DX_MIN_Z = -8.5;

	static const float DX_MAX_X = -17;
	static const float DX_MAX_Y = -37;
	static const float DX_MAX_Z = -6.5;


	static const float CABLE_MIN_X = -35;
	static const float CABLE_MIN_Y = -60;
	static const float CABLE_MIN_Z = -15;

	static const float CABLE_MAX_X = 0;
	static const float CABLE_MAX_Y = -30;
	static const float CABLE_MAX_Z = -2;

};


#endif /* EXTRACT_OBJECT_H_ */
