
#include "extract_object.h"


using namespace std;
using namespace pcl;


PointCloudPtr ExtractObject::extractPioli(PointCloudPtr cloud){

	cout << "Estraggo pioli" << endl;
	PointCloudPtr pioli(new PointCloud<PointXYZRGB>);


	PointCloud<PointXYZRGB>::Ptr cloud_clean (new PointCloud<PointXYZRGB>);
	ConditionAnd<PointXYZRGB>::Ptr range_cond (new ConditionAnd<PointXYZRGB> ());
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("x", ComparisonOps::GE, -30)));
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("x", ComparisonOps::LE, -15)));

	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("y", ComparisonOps::GE, -43)));
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("y", ComparisonOps::LE, -37)));

	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
				(new FieldComparison<PointXYZRGB> ("z", ComparisonOps::LE, -7)));
	ConditionalRemoval<PointXYZRGB> condrem (range_cond);
	condrem.setInputCloud (cloud);
	condrem.filter (*cloud_clean);

	Utils::setColor(cloud_clean, 0, 0, 255);
	CloudIO::visualize(cloud, "cloud", cloud_clean, "zona interesse pioli");


	cout << "Clusterizzazione" << endl;
	vector<PointIndices> cluster_indices = ExtractObject::clusterization(cloud_clean, 3, 1, 100);

	cout << "Cerco cluster pioli" << endl;
	cout << "#cluster: " << cluster_indices.size() << endl;
	int j = 0;
	for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		PointCloudPtr cloud_cluster (new PointCloud<pcl::PointXYZRGB>);
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_clean->points[*pit]);

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;


		if (ExtractObject::isPioloSx(cloud_cluster) || ExtractObject::isPioloDx(cloud_cluster)){
			Utils::setColor(cloud_cluster, 0, 255, 0);
			Utils::copyTo(cloud_cluster, pioli);
		}else{
			Utils::setColor(cloud_cluster, 255, 255, 0);
			Utils::copyTo(cloud_cluster, pioli);
		}
		j++;
	}


	//********************
	PointXYZRGB p;
	p.x = SX_MIN_X; p.y = SX_MIN_Y; p.z = SX_MIN_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);

	p.x = SX_MAX_X; p.y = SX_MIN_Y; p.z = SX_MIN_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);

	p.x = SX_MAX_X; p.y = SX_MAX_Y; p.z = SX_MIN_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);

	p.x = SX_MIN_X; p.y = SX_MAX_Y; p.z = SX_MIN_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);

	p.x = SX_MIN_X; p.y = SX_MIN_Y; p.z = SX_MAX_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);

	p.x = SX_MAX_X; p.y = SX_MIN_Y; p.z = SX_MAX_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);

	p.x = SX_MAX_X; p.y = SX_MAX_Y; p.z = SX_MAX_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);

	p.x = SX_MIN_X; p.y = SX_MAX_Y; p.z = SX_MAX_Z;
	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);


	//dx
	p.x = DX_MIN_X; p.y = DX_MIN_Y; p.z = DX_MIN_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	p.x = DX_MAX_X; p.y = DX_MIN_Y; p.z = DX_MIN_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	p.x = DX_MAX_X; p.y = DX_MAX_Y; p.z = DX_MIN_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	p.x = DX_MIN_X; p.y = DX_MAX_Y; p.z = DX_MIN_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	p.x = DX_MIN_X; p.y = DX_MIN_Y; p.z = DX_MAX_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	p.x = DX_MAX_X; p.y = DX_MIN_Y; p.z = DX_MAX_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	p.x = DX_MAX_X; p.y = DX_MAX_Y; p.z = DX_MAX_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	p.x = DX_MIN_X; p.y = DX_MAX_Y; p.z = DX_MAX_Z;
	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	//********************

	return pioli;
}


PointCloudPtr ExtractObject::extractCable(PointCloudPtr cloud){

	PointCloud<PointXYZRGB>::Ptr cloud_clean (new PointCloud<PointXYZRGB>);
	ConditionAnd<PointXYZRGB>::Ptr range_cond (new ConditionAnd<PointXYZRGB> ());

	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("x", ComparisonOps::GE, -31)));
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("x", ComparisonOps::LE, 0)));

	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
				(new FieldComparison<PointXYZRGB> ("y", ComparisonOps::LE, -35)));

	ConditionalRemoval<PointXYZRGB> condrem (range_cond);
	condrem.setInputCloud (cloud);
	condrem.filter (*cloud_clean);


	Utils::setColor(cloud_clean, 0, 0, 255);
	CloudIO::visualize(cloud, "cloud", cloud_clean, "zona interesse cavo");
	cloud = cloud_clean;

	cout << "Estraggo cavo" << endl;
	PointCloudPtr cable(new PointCloud<PointXYZRGB>);


	cout << "Clusterizzazione" << endl;
	vector<PointIndices> cluster_indices = ExtractObject::clusterization(cloud, 1, 250, 55000);

	cout << "Cerco cluster cavo" << endl;
	int j = 0;
	int r = 32, g = 64, b = 127;
	for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		PointCloudPtr cloud_cluster (new PointCloud<pcl::PointXYZRGB>);
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]);

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;


		if (ExtractObject::isCable(cloud_cluster)){
			Utils::setColor(cloud_cluster, 255, 0, 0);
			Utils::copyTo(cloud_cluster, cable);
		}else{
			Utils::setColor(cloud_cluster, 255*rand(), 255*rand(), 255*rand());
			Utils::copyTo(cloud_cluster, cable);
		}
		j++;

//		bool isCable = ExtractObject::isCable(cloud_cluster);
//		for (int i = 0; i < cloud_cluster->width; i++){
//			PointXYZRGB p;
//			p.x = cloud_cluster->at(i).x;
//			p.y = cloud_cluster->at(i).y;
//			p.z = cloud_cluster->at(i).z;
//
//			if (isCable){
//				p.r = 255;
//				p.g = 0;
//				p.b = 0;
//				cable->points.push_back(p);
//			}else{
//				p.r = 0;
//				p.g = g;
//				p.b = b;
////				cable->points.push_back(p);
//			}
////
//		}
//		r = (r+27)%256; g = (g+94)%256; b = (b+63)%256;
//		j++;
	}


	//********************
	PointXYZRGB p;
	p.x = CABLE_MIN_X; p.y = CABLE_MIN_Y; p.z = CABLE_MIN_Z;
	p.r = 0; p.g = 0; p.b = 255; cable->push_back(p);

	p.x = CABLE_MAX_X; p.y = CABLE_MIN_Y; p.z = CABLE_MIN_Z;
	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);

	p.x = CABLE_MAX_X; p.y = CABLE_MAX_Y; p.z = CABLE_MIN_Z;
	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);

	p.x = CABLE_MIN_X; p.y = CABLE_MAX_Y; p.z = CABLE_MIN_Z;
	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);

	p.x = CABLE_MIN_X; p.y = CABLE_MIN_Y; p.z = CABLE_MAX_Z;
	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);

	p.x = CABLE_MAX_X; p.y = CABLE_MIN_Y; p.z = CABLE_MAX_Z;
	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);

	p.x = CABLE_MAX_X; p.y = CABLE_MAX_Y; p.z = CABLE_MAX_Z;
	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);

	p.x = CABLE_MIN_X; p.y = CABLE_MAX_Y; p.z = CABLE_MAX_Z;
	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
	/*****************/

	return cable;
}

bool ExtractObject::isCable(PointCloudPtr cluster){

	int inlier = 0;
	for (int i = 0; i <cluster->width; i++){
		PointXYZRGB p;
		p.x = cluster->at(i).x;
		p.y = cluster->at(i).y;
		p.z = cluster->at(i).z;

		if (/*cluster->width > 90
				&& cluster->width < 550
				&&*/ p.x > CABLE_MIN_X && p.x < CABLE_MAX_X
				&& p.y > CABLE_MIN_Y && p.y < CABLE_MAX_Y
				&& p.z > CABLE_MIN_Z && p.z < CABLE_MAX_Z){
			inlier++;
		}
	}
	cout << "cable inlier: " << inlier << " su " << cluster->width << " " << (float)inlier/cluster->points.size() << endl;
	return (float)inlier/cluster->width > 0.7;

}


bool ExtractObject::isPioloSx(PointCloudPtr cloud){

	int inlier = 0;
	for (int i = 0; i < cloud->points.size(); i++){
		PointXYZRGB p = cloud->points[i];
		if (p.x > SX_MIN_X && p.x < SX_MAX_X
				&& p.y > SX_MIN_Y && p.y < SX_MAX_Y
				&& p.z > SX_MIN_Z && p.z < SX_MAX_Z){
			inlier++;
		}
	}
	cout << "pioloSx inlier: " << inlier << " su " << cloud->width << " " << (float)inlier/cloud->points.size() << endl;
	return (float)inlier/cloud->points.size() > 0.7;

}


bool ExtractObject::isPioloDx(PointCloudPtr cloud){

	int inlier = 0;
	for (int i = 0; i < cloud->points.size(); i++){
		PointXYZRGB p = cloud->points[i];
		if (p.x > -22 && p.x < -16
				&& p.y > -42 && p.y < -36
				&& p.z > -8.5 && p.z < -6.5){
			inlier ++;
		}
	}
	cout << "piolodx inlier: " << inlier << " su " << cloud->width << " " << (float)inlier/cloud->points.size() << endl;
	return (float)inlier/cloud->points.size() > 0.7;

}


PointCloudPtr ExtractObject::segmentation(PointCloudPtr cloud, double threshold){

	pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::PointCloud<PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB> ());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Mandatory
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setDistanceThreshold (threshold);


	cloud_filtered = cloud;
	int i=0, nr_points = (int) cloud_filtered->points.size ();
	while (cloud_filtered->points.size () > 0.7 * nr_points){
		seg.setInputCloud((*cloud_filtered).makeShared());
		seg.segment (*inliers, *coefficients);

		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices(inliers);

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered = cloud_f;
	}

	return cloud_filtered;

}


vector<PointIndices> ExtractObject::clusterization(PointCloudPtr cloud, double tolerance, double min, double max){
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min);
    ec.setMaxClusterSize (max);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    return cluster_indices;
}


bool ExtractObject::isCableCorrect(PointCloudPtr pioli, PointCloudPtr cable){
	double maxX, maxY, maxZ;
	double minX, minY, minZ;

	maxX = maxY = maxZ = -INFINITY;
	minX = minY = minZ = +INFINITY;
	maxZ = -4;

	for(int i = 0; i < pioli->points.size(); i++){
		PointXYZRGB p = pioli->points[i];
		if (p.x > maxX)
			maxX = p.x;
		if (p.x < minX)
			minX = p.x;

		if (p.y > maxY)
			maxY = p.y;
		if (p.y < minY)
			minY = p.y;

//		if (p.z > maxZ)
//			maxZ = p.z;
		if (p.z < minZ)
			minZ = p.z;
	}

	int inliers = 0;
	for(int i = 0; i < cable->points.size(); i++){
		PointXYZRGB p = cable->points[i];
//		cout << p.x << " " << p.y << " " << p.z << endl;
		if (p.x > minX && p.x < maxX
				&& p.y > minY && p.y < maxY
				&& p.z > minZ && p.z < maxZ){
			inliers++;
		}
	}

	cout << minX << " " << maxX << endl;
	cout << minY << " " << maxY << endl;
	cout << minZ << " " << maxZ << endl;

	cout << inliers << " " << cable->points.size() << endl;
	cout << (float)inliers/(float)cable->points.size() << endl;

	return inliers > 350;
}




