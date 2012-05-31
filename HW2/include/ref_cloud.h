/*
 * RefCloud.h
 *
 * In questa classe viene memorizzati tutti i dati riguardanti la classe di riferimento
 */

#ifndef REFCLOUD_H_
#define REFCLOUD_H_

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/sift_keypoint.h>

#include "my_datatype.h"
#include "cloud_io.h"


class RefCloud{


public:

	/**
	 * Costruttore, carica la point cloud di riferimento, calcola i keypoint
	 * 		SIFT e le feature FPFH. Mantiene anche una versione filtrata con Voxel.
	 * path: percorso in cui si trova la point cloud di riferimento
	 */
	RefCloud(const std::string& path);

	/**
	 * Applica un filtro voxel
	 */
	static PointCloudPtr filterVoxel(const PointCloudPtr& cloud, float lx, float ly, float lz);
	static PointCloudPtr filterVoxel(const PointCloudPtr& cloud, float l);

	/**
	 * Registra la point cloud input sulla point cloud di riferimento
	 */
	PointCloudPtr registration(const PointCloudPtr& input) const;


private:
	//Registra la point cloud usando SAC
	PointCloudPtr alignSAC(const PointCloudPtr& inputKeypoint, const FeatureCloudPtr& inputFeature,
						   Eigen::Matrix4f& traformation) const;

	//Registra la point cloud usando ICP (dopo averla registrata con SAC)
	PointCloudPtr alignICP(const PointCloudPtr& alignedSAC, Eigen::Matrix4f& traformation) const;

	//Calcola i keypoint SIFT
	static PointCloudPtr computeSIFT(const PointCloudPtr& cloud);

	//Calcolo la feature FPFH
	static FeatureCloudPtr computeFPFH(const PointCloudPtr& cloud, const PointCloudPtr& filtered);


private:
	PointCloudPtr 	cloud;		//cloud originale
	PointCloudPtr	filtered;	//cloud filtrata (voxel)
	PointCloudPtr 	keypoint;	//cloud dei keypoint
	FeatureCloudPtr feature;	//cloud feature dei keypoint


public:
	static const float VOXEL_LEAF_SIZE = 0.9;


};



#endif /* REFCLOUD_H_ */
