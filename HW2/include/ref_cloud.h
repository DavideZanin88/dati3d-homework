/*
 * RefCloud.h
 *
 * In questa classe viene memorizzati tutti i dati riguardanti la classe di riferimento
 */

#ifndef REFCLOUD_H_
#define REFCLOUD_H_

#include <pcl/common/common.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

#include "my_datatype.h"
#include "cloud_io.h"
#include "utils.h"


class RefCloud{


public:

	/**
	 * Costruttore, carica la point cloud di riferimento, calcola i keypoint
	 * 		SIFT e le feature FPFH. Mantiene anche una versione filtrata con Voxel.
	 * path: percorso in cui si trova la point cloud di riferimento
	 */
	RefCloud(const std::string& path);


	/**
	 * Registra la point cloud input sulla point cloud di riferimento
	 */
	PointCloudPtr registration(PointCloudPtr input);


private:
	PointCloudPtr alignSAC(PointCloudPtr inputKeypoint, FeatureCloudPtr inputFeature,
						   Eigen::Matrix4f& traformation);

	PointCloudPtr alignICP(PointCloudPtr alignedSAC, Eigen::Matrix4f& traformation);


private:
	PointCloudPtr 	cloud;		//cloud originale
	PointCloudPtr	filtered;	//cloud filtrata (voxel)
	PointCloudPtr 	keypoint;	//cloud dei keypoint
	FeatureCloudPtr feature;	//cloud feature dei keypoint


public:
	static const float VOXEL_LEAF_SIZE = 0.7;



};



#endif /* REFCLOUD_H_ */
