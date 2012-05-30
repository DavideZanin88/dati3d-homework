
#include "utils.h"


using namespace std;
using namespace pcl;


PointCloudPtr Utils::voxel(const PointCloudPtr& cloud, float lx, float ly, float lz){
	PointCloudPtr filtered(new PointCloud<PointXYZRGB>);
	VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (lx, ly, lz);
	sor.filter (*filtered);
	return filtered;
}


PointCloudPtr Utils::voxel(const PointCloudPtr& cloud, float l){
	return Utils::voxel(cloud, l, l, l);
}


FeatureCloudPtr Utils::computeFPFH(const PointCloudPtr& cloud, const PointCloudPtr& keypoint){

	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

	NormalEstimationOMP<PointXYZRGB, Normal> ne;
	ne.setInputCloud (cloud);
	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
	ne.setSearchMethod (tree);
	ne.setNumberOfThreads(4);
	ne.setRadiusSearch (6);

	cout << "Calcolo le normali...";
	ne.compute (*normals);
	cout << " OK!" << endl;

	//calcolo le FPFH
	FeatureCloudPtr fpfhs (new PointCloud<FPFHSignature33> ());
	FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33> fpfh;
	fpfh.setInputCloud(keypoint);
	fpfh.setInputNormals(normals);
	fpfh.setSearchSurface(cloud);
	fpfh.setNumberOfThreads(4);
	// NOTA deve essere maggiore del valore messo nel setRadiusSearch del NormalEstimation!!!
	fpfh.setRadiusSearch (6.75);

	// Compute the features
	std::cout << "Calcolo le feature FPFH...";
	fpfh.compute (*fpfhs);
	std::cout << " OK!" << std::endl;

	return fpfhs;
}


PointCloudPtr Utils::computeSIFT(const PointCloudPtr& cloud){
	PointCloudPtr keypoints(new PointCloud<PointXYZRGB>);
	SIFTKeypoint<PointXYZRGB, PointXYZRGB> sift;

	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
	sift.setSearchMethod(tree);
	sift.setScales (0.05, 3, 5);
	sift.setMinimumContrast (0.5);
	sift.setInputCloud (cloud);

	// Detect the keypoints and store them in "keypoints_out"
	cout << "Calcolo SIFTKeypoint...";
	sift.compute (*keypoints);
	cout << " OK!" << endl;
	return keypoints;
}


void Utils::setColor(const PointCloudPtr& cloud, char r, char g, char b){
	for (int i = 0; i < cloud->points.size(); i++){
		cloud->points[i].r = r;
		cloud->points[i].g = g;
		cloud->points[i].b = b;
	}
}


void Utils::copyTo(const PointCloudPtr& src, const PointCloudPtr& dest){
	for (int i = 0; i < src->points.size(); i++){
		dest->points.push_back(src->points[i]);
	}
	dest->width = dest->points.size ();
	dest->height = 1;
	dest->is_dense = true;
}






