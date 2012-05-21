
#include "extract_object.h"


using namespace std;
using namespace pcl;


PointCloudPtr ExtractObject::extractPioli(PointCloudPtr cloud){

	cout << "Estraggo pioli" << endl;
	PointCloudPtr pioli(new PointCloud<PointXYZRGB>);
	cout << "Segmentazione" << endl;
//	PointCloudPtr segmentation = ExtractObject::segmentation(cloud, 1);
	cout << "Clusterizzazione" << endl;
	vector<PointIndices> cluster_indices = ExtractObject::clusterization(cloud, 0.7, 10, 30);

	cout << "Cerco cluster pioli" << endl;
	int j = 0;
	for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		PointCloudPtr cloud_cluster (new PointCloud<pcl::PointXYZRGB>);
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]);

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		for (int i = 0; i < cloud_cluster->width; i++){
			PointXYZRGB p;
			p.x = cloud_cluster->at(i).x;
			p.y = cloud_cluster->at(i).y;
			p.z = cloud_cluster->at(i).z;

			if (cloud_cluster->width < 30
					&& p.x > -27 && p.x < -23
					&& p.y > -42 && p.y < -38
					&& p.z > -9.5 && p.z < -7.5){
				p.r = 0;
				p.g = 255;
				p.b = 0;
				pioli->points.push_back(p);
			}

			if (cloud_cluster->width < 30
					&& p.x > -22 && p.x < -18
					&& p.y > -43 && p.y < -39
					&& p.z > -9.5 && p.z < -7.5){
				p.r = 0;
				p.g = 255;
				p.b = 0;
				pioli->points.push_back(p);
			}
		}
		j++;
	}

	return pioli;
}

PointCloudPtr ExtractObject::extractCable(PointCloudPtr cloud){

		cout << "Segmentazione" << endl;
		PointCloudPtr segmentation = ExtractObject::segmentation(cloud, 0.05);
		CloudIO::visualize(segmentation, "segmentata");
		cloud = segmentation;

	PointCloud<PointXYZRGB>::Ptr cloud_clean (new PointCloud<PointXYZRGB>);
	ConditionAnd<PointXYZRGB>::Ptr range_cond (new ConditionAnd<PointXYZRGB> ());
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("y", ComparisonOps::LE, -35)));
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
				(new FieldComparison<PointXYZRGB> ("x", ComparisonOps::GE, -30)));
	ConditionalRemoval<PointXYZRGB> condrem (range_cond);
	condrem.setInputCloud (cloud);
	condrem.filter (*cloud_clean);
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

		bool isCable = ExtractObject::isCable(cloud_cluster);
		for (int i = 0; i < cloud_cluster->width; i++){
			PointXYZRGB p;
			p.x = cloud_cluster->at(i).x;
			p.y = cloud_cluster->at(i).y;
			p.z = cloud_cluster->at(i).z;

			if (isCable){
				p.r = 255;
				p.g = 0;
				p.b = 0;
				cable->points.push_back(p);
			}else{
				p.r = 0;
				p.g = g;
				p.b = b;
//				cable->points.push_back(p);
			}
//
		}
		r = (r+27)%256; g = (g+94)%256; b = (b+63)%256;
		j++;
	}

	return cable;
}

bool ExtractObject::isCable(PointCloudPtr cluster){

	int inliers = 0;
	for (int i = 0; i <cluster->width; i++){
		PointXYZRGB p;
		p.x = cluster->at(i).x;
		p.y = cluster->at(i).y;
		p.z = cluster->at(i).z;

		if (/*cluster->width > 90
				&& cluster->width < 550
				&&*/ p.x > -30 && p.x < 5
				&& p.y > -55 && p.y < -35
				&& p.z > -15 && p.z < -2){
			inliers++;
		}
	}
	return (float)inliers/(float)cluster->width > 0.8;

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

	return (float)inliers/(float)cable->points.size() >= 0.05;
}




