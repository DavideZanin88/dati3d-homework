
#include "registration.h"


using namespace pcl;

PointCloudPtr myRegistration::registration(PointCloudPtr src, PointCloudPtr target){

	//esegue un voxel
	PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr filt(new PointCloud<PointXYZRGB>);
	VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud (src);
	sor.setLeafSize (0.4f, 0.4f, 0.4f);
	sor.filter (*cloud_filtered);
	src = cloud_filtered;

	PointCloud<PointXYZRGB>::Ptr cloud_filtered2(new PointCloud<PointXYZRGB>);
	sor.setInputCloud (target);
	sor.setLeafSize (0.4f, 0.4f, 0.4f);
	sor.filter (*cloud_filtered2);
	target = cloud_filtered2;


	//calcola i keypoint
	PointCloudPtr srcKey = myRegistration::detectKeypoints(src);
	PointCloudPtr targetKey = myRegistration::detectKeypoints(target);


	//calcola le feature
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr srcFPFH = findFPFH(src, srcKey);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFPFH = findFPFH(target, targetKey);


	//eseue il primo alineamento (basato su feature)
	cout << "Eseguo SAC...";
	PointCloudPtr aligned(new PointCloud<PointXYZRGB>);
	SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac;
	sac.setInputCloud(srcKey);
	sac.setSourceFeatures(srcFPFH);
	sac.setInputTarget(targetKey);
	sac.setTargetFeatures(targetFPFH);

	sac.setMinSampleDistance(10);			//TODO sperimentare
	sac.setMaximumIterations(100);			// un po' con questi
//	sac.setMaxCorrespondenceDistance(150);	// parametri
	sac.setTransformationEpsilon(0.2);

	sac.align(*aligned);
	cout << " OK!" << endl;

	//stampa info sull'alineamento e visualizza
	cout << " score: " << sac.getFitnessScore() << endl;
	cout << sac.getFinalTransformation() << endl;
	CloudIO::visualize(target, "target", aligned, "aligned");


	//rifinisci l'alineamento usanto icp
	cout << "Eseguo icp..." << endl;
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(aligned);
	icp.setInputTarget(target);
	icp.setMaxCorrespondenceDistance(100);		//TODO sperimentare!
	icp.setMaximumIterations(500);
	icp.setTransformationEpsilon(0.0005);
	icp.setRANSACOutlierRejectionThreshold(0.3); //-----

	PointCloudPtr aligned2(new PointCloud<PointXYZRGB>);
	icp.align(*aligned2);
	CloudIO::visualize(target, "target", aligned2, "aligned2");
	cout << " OK!" << endl;

	//stampa info sull'alineamento e visualizza
	cout << " score: " << icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;


	PointCloud<PointXYZRGB>::Ptr final_aligned(new PointCloud<PointXYZRGB>);
	transformPointCloud(*src, *final_aligned, icp.getFinalTransformation()*sac.getFinalTransformation());
	CloudIO::visualize(final_aligned, "final_aligned");

	return final_aligned;

}



PointCloud<FPFHSignature33>::Ptr myRegistration::findFPFH(PointCloudPtr cloud, PointCloudPtr filt){

	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

	NormalEstimationOMP<PointXYZRGB, Normal> ne;
	ne.setInputCloud (cloud);
	search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
	ne.setSearchMethod (tree);
	ne.setNumberOfThreads(4);
	ne.setRadiusSearch (5); //TODO 5 sembra buono ma con le nuove modifiche forse si può diminuire :)

	cout << "Calcolo le normali...";
	ne.compute (*cloud_normals);
	cout << " OK!" << endl;


	//calcolo le FPFH
	PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());
	FPFHEstimationOMP<PointXYZRGB, Normal, FPFHSignature33> fpfh;
	fpfh.setInputCloud(filt);
	fpfh.setInputNormals(cloud_normals);
	fpfh.setSearchSurface(cloud);
	fpfh.setNumberOfThreads(4);
	// NOTA deve essere maggiore del valore messo nel setRadiusSearch del NormalEstimation!!!
	fpfh.setRadiusSearch (6);

	// Compute the features
	std::cout << "Calcolo le feature FPFH...";
	fpfh.compute (*fpfhs);
	std::cout << " OK!" << std::endl;

	return fpfhs;
}



PointCloudPtr myRegistration::detectKeypoints(PointCloudPtr cloud){
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
