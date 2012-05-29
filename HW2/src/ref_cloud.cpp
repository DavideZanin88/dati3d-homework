
#include "ref_cloud.h"


using namespace std;
using namespace pcl;


RefCloud::RefCloud(const string& path){

	cout << "Carico la point cloud di riferimento: " << path << endl;
	this->cloud = CloudIO::loadPointCloud(path);
	this->filtered = Utils::voxel(this->cloud, RefCloud::VOXEL_LEAF_SIZE);
	this->keypoint = Utils::computeSIFT(this->filtered);
	this->feature = Utils::computeFPFH(this->filtered, this->keypoint);
	cout << "Point cloud di riferimento caricata con successo!" << endl;

}


PointCloudPtr RefCloud::registration(const PointCloudPtr& input) const{

	//esegue un voxel
	PointCloudPtr filtered = Utils::voxel(input, RefCloud::VOXEL_LEAF_SIZE);

	//calcola i keypoint
	PointCloudPtr inputKeypoint = Utils::computeSIFT(filtered);

	//calcola le feature
	FeatureCloudPtr inputFeature = Utils::computeFPFH(filtered, inputKeypoint);

	//alinea con SAC e rifinisci con ICP
	Eigen::Matrix4f trasfSAC, trasfICP;
	PointCloudPtr aligned = RefCloud::alignSAC(inputKeypoint, inputFeature, trasfSAC);
	aligned = RefCloud::alignICP(aligned, trasfICP);

	//registra la cloud non filtrata
	PointCloud<PointXYZRGB>::Ptr input_aligned(new PointCloud<PointXYZRGB>);
	transformPointCloud(*input, *input_aligned, trasfICP*trasfSAC);

//	CloudIO::visualize(input_aligned, "final_aligned");

	return input_aligned;

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
	sac.setMaximumIterations(750);
	sac.setTransformationEpsilon(0.0001);

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

	icp.setMaxCorrespondenceDistance(100);
	icp.setMaximumIterations(125);
	icp.setTransformationEpsilon(0.005);
	icp.setRANSACOutlierRejectionThreshold(0.3);
	icp.align(*aligned);
	traformation = icp.getFinalTransformation();

	cout << " OK! score: " << icp.getFitnessScore() << endl;
//	CloudIO::visualize(target, "target", aligned, "aligned icp");

	return aligned;
}







