
#include "ref_cloud.h"


using namespace std;
using namespace pcl;


RefCloud::RefCloud(const string& path){

	cout << "Carico la point cloud di riferimento: " << path << endl;
	this->cloud = CloudIO::loadPointCloud(path);
	this->filtered = RefCloud::filterVoxel(this->cloud, RefCloud::VOXEL_LEAF_SIZE);
	this->keypoint = RefCloud::computeSIFT(this->filtered);
	this->feature = RefCloud::computeFPFH(this->filtered, this->keypoint);
	cout << "Point cloud di riferimento caricata con successo!" << endl;

}


PointCloudPtr RefCloud::filterVoxel(const PointCloudPtr& cloud, float lx, float ly, float lz){
	PointCloudPtr filtered(new PointCloud<PointXYZRGB>);
	VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (lx, ly, lz);
	sor.filter (*filtered);
	return filtered;
}


PointCloudPtr RefCloud::filterVoxel(const PointCloudPtr& cloud, float l){
	return RefCloud::filterVoxel(cloud, l, l, l);
}


PointCloudPtr RefCloud::registration(const PointCloudPtr& input) const{

	//esegue un voxel
	PointCloudPtr inputFiltered = RefCloud::filterVoxel(input, RefCloud::VOXEL_LEAF_SIZE);

	//calcola i keypoint
	PointCloudPtr inputKeypoint = RefCloud::computeSIFT(inputFiltered);

	//calcola le feature
	FeatureCloudPtr inputFeature = RefCloud::computeFPFH(inputFiltered, inputKeypoint);

	//alinea con SAC e rifinisci con ICP
	Eigen::Matrix4f trasfSAC, trasfICP;
	PointCloudPtr aligned = RefCloud::alignSAC(inputKeypoint, inputFeature, trasfSAC);
	PointCloudPtr inputAlignedSAC(new PointCloud<PointXYZRGB>);
	transformPointCloud(*inputFiltered, *inputAlignedSAC, trasfSAC);
	aligned = RefCloud::alignICP(inputAlignedSAC, trasfICP);

	//registra la cloud non filtrata
	PointCloud<PointXYZRGB>::Ptr inputFinalAligned(new PointCloud<PointXYZRGB>);
	transformPointCloud(*input, *inputFinalAligned, trasfICP*trasfSAC);

//	CloudIO::visualize(input_aligned, "final_aligned");

	return inputFinalAligned;

}


PointCloudPtr RefCloud::alignSAC(const PointCloudPtr& inputKeypoint, const FeatureCloudPtr& inputFeature,
								 Eigen::Matrix4f& traformation) const{

	//eseue il primo alineamento (basato su feature)
	cout << "Eseguo SAC...";
	PointCloudPtr aligned(new PointCloud<PointXYZRGB>);
	SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac;
	sac.setInputCloud(inputKeypoint);
	sac.setSourceFeatures(inputFeature);
	sac.setInputTarget(this->keypoint);
	sac.setTargetFeatures(this->feature);

	sac.setMinSampleDistance(35);
	sac.setMaximumIterations(400);
	sac.setTransformationEpsilon(0.001);

	sac.align(*aligned);
	traformation = sac.getFinalTransformation();

	cout << " OK! score: " << sac.getFitnessScore() << endl;
	//	CloudIO::visualize(target, "target", aligned, "aligned SAC");

	return aligned;
}


PointCloudPtr RefCloud::alignICP(const PointCloudPtr& alignedSAC, Eigen::Matrix4f& traformation) const{
	//rifinisci l'alineamento usanto icp
	cout << "Eseguo icp...";
	PointCloudPtr aligned(new PointCloud<PointXYZRGB>);
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp = IterativeClosestPoint<PointXYZRGB, PointXYZRGB>();
	icp.setInputCloud(alignedSAC);
	icp.setInputTarget(this->filtered);

	icp.setMaxCorrespondenceDistance(40);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(0.01);
	icp.setRANSACOutlierRejectionThreshold(50);
	icp.align(*aligned);
	traformation = icp.getFinalTransformation();

	cout << " OK! score: " << icp.getFitnessScore() << endl;
//	CloudIO::visualize(target, "target", aligned, "aligned icp");

	return aligned;
}


PointCloudPtr RefCloud::computeSIFT(const PointCloudPtr& cloud){
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


FeatureCloudPtr RefCloud::computeFPFH(const PointCloudPtr& cloud, const PointCloudPtr& keypoint){

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






