
#include "utils.h"


using namespace std;
using namespace pcl;


PointCloudPtr Utils::voxel(PointCloudPtr cloud, float lx, float ly, float lz){
	PointCloudPtr filtered(new PointCloud<PointXYZRGB>);
	VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (lx, ly, lz);
	sor.filter (*filtered);
	return filtered;
}


PointCloudPtr Utils::voxel(PointCloudPtr cloud, float l){
	PointCloudPtr filtered(new PointCloud<PointXYZRGB>);
	VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (l, l, l);
	sor.filter (*filtered);
	return filtered;
}


FeatureCloudPtr Utils::computeFPFH(PointCloudPtr cloud, PointCloudPtr keypoint){

	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

	NormalEstimationOMP<PointXYZRGB, Normal> ne;
	ne.setInputCloud (cloud);
	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
	ne.setSearchMethod (tree);
	ne.setNumberOfThreads(4);
	ne.setRadiusSearch (6); //TODO 5 sembra buono ma con le nuove modifiche forse si può diminuire :)

	cout << "Calcolo le normali...";
	ne.compute (*cloud_normals);
	cout << " OK!" << endl;

	//calcolo le FPFH
	PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());
	FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33> fpfh;
	fpfh.setInputCloud(keypoint);
	fpfh.setInputNormals(cloud_normals);
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


PointCloudPtr Utils::computeSIFT(PointCloudPtr cloud){
	PointCloud<PointXYZRGB>::Ptr keypoints_out(new PointCloud<PointXYZRGB>);
	SIFTKeypoint<PointXYZRGB, PointXYZRGB> sift_detect;

	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
	sift_detect.setSearchMethod(tree);
	sift_detect.setScales (0.05, 3, 5);     //TODO questi 4 parametri non li ho mai cambiati
	sift_detect.setMinimumContrast (0.5);	// e non ho idea di come vadano impostati!
	sift_detect.setInputCloud (cloud);

	// Detect the keypoints and store them in "keypoints_out"
	cout << "Calcolo SIFTKeypoint...";
	sift_detect.compute (*keypoints_out);
	cout << " OK!" << endl;
	return keypoints_out;
}


void Utils::setColor(PointCloudPtr cloud, char r, char g, char b){
	for (int i = 0; i < cloud->points.size(); i++){
		cloud->points[i].r = r;
		cloud->points[i].g = g;
		cloud->points[i].b = b;
	}
}


void Utils::copyTo(const PointCloudPtr src, PointCloudPtr dest){
	for (int i = 0; i < src->points.size(); i++){
		dest->points.push_back(src->points[i]);
	}
	dest->width = dest->points.size ();
	dest->height = 1;
	dest->is_dense = true;
}






