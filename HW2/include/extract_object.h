
#ifndef EXTRACT_OBJECT_H_
#define EXTRACT_OBJECT_H_


#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "my_datatype.h"
#include "cloud_io.h"


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

	/** Ritorna true se almeno il 70% dei punti di cloud si trova all'interno del
	 *  parallelepipedo individuato da min e max*/
	static bool isIn(const PointCloudPtr& cloud, pcl::PointXYZ min, pcl::PointXYZ max);

	/** Imposta il colore dei punti di cloud **/
	static void setColor(const PointCloudPtr& cloud, char r, char g, char b);

	/** Copia i punti di src in dest **/
	static void copyTo(const PointCloudPtr& src, const PointCloudPtr& dest);


public:

	/** Coordinate zona in cui cercare piolo sx **/
	static const pcl::PointXYZ PSX_MIN;
	static const pcl::PointXYZ PSX_MAX;

	/** Coordinate zona in cui cercare piolo dx **/
	static const pcl::PointXYZ PDX_MIN;
	static const pcl::PointXYZ PDX_MAX;

	/** Coordinate zona in cui cercare il cavo **/
	static const pcl::PointXYZ CABLE_MIN;
	static const pcl::PointXYZ CABLE_MAX;

};


#endif /* EXTRACT_OBJECT_H_ */
