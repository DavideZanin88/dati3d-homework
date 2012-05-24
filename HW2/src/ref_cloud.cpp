
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


PointCloudPtr RefCloud::registration(PointCloudPtr input){

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

	CloudIO::visualize(input_aligned, "final_aligned");

	return input_aligned;

}


PointCloudPtr RefCloud::alignSAC(PointCloudPtr inputKeypoint, FeatureCloudPtr inputFeature,
								 Eigen::Matrix4f& traformation){

	//eseue il primo alineamento (basato su feature)
	cout << "Eseguo SAC...";
	PointCloudPtr aligned(new PointCloud<PointXYZRGB>);
	SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac;
	sac.setInputCloud(inputKeypoint);
	sac.setSourceFeatures(inputFeature);
	sac.setInputTarget(this->keypoint);
	sac.setTargetFeatures(this->feature);

	sac.setMinSampleDistance(40);			//TODO sperimentare
	sac.setMaximumIterations(750);			// un po' con questi
	//	sac.setMaxCorrespondenceDistance(150);	// parametri
	sac.setTransformationEpsilon(0.0001);

	sac.align(*aligned);
	cout << " OK!" << endl;

	//stampa info sull'alineamento e visualizza
	cout << " score: " << sac.getFitnessScore() << endl;
	cout << sac.getFinalTransformation() << endl;
	//	CloudIO::visualize(target, "target", aligned, "aligned");

	traformation = sac.getFinalTransformation();
	return aligned;
}


PointCloudPtr RefCloud::alignICP(PointCloudPtr alignedSAC, Eigen::Matrix4f& traformation){ //TODO double score
	//rifinisci l'alineamento usanto icp
	cout << "Eseguo icp..." << endl;
	PointCloudPtr aligned2(new PointCloud<PointXYZRGB>);
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	do{
		icp = IterativeClosestPoint<PointXYZRGB, PointXYZRGB>();
		icp.setInputCloud(alignedSAC);
		icp.setInputTarget(this->filtered);
		icp.setMaxCorrespondenceDistance(150);		//TODO sperimentare!
		icp.setMaximumIterations(300);
		icp.setTransformationEpsilon(0.0005);
		icp.setRANSACOutlierRejectionThreshold(0.3); //-----
		icp.align(*aligned2);
		if (icp.getFitnessScore() > 1.5)
			cout << " ritento " << icp.getFitnessScore() << "\n";
	}while (icp.getFitnessScore() > 1.5); //TODO if
	cout << " OK!" << endl;

	//stampa info sull'alineamento e visualizza
	cout << " score: " << icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;
//	CloudIO::visualize(target, "target", aligned2, "aligned2");

	traformation = icp.getFinalTransformation();
	return aligned2;
}







