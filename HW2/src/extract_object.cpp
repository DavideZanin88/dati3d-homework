
#include "extract_object.h"


using namespace std;
using namespace pcl;


PointCloudPtr ExtractObject::extractPioli(const PointCloudPtr& cloud){

	cout << "Estraggo pioli" << endl;
	PointCloudPtr pioli(new PointCloud<PointXYZRGB>);

	//"ritaglia" la poin cloud tenendo solo la parte in cui si trovano i pioli
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

//	Utils::setColor(cloud_clean, 0, 0, 255);
//	CloudIO::visualize(cloud, "cloud", cloud_clean, "zona pioli");

	cout << "Clusterizzazione" << endl;
	vector<PointIndices> indices = ExtractObject::clusterization(cloud_clean, 0.5, 1, 100);

	cout << "Cerco i cluster dei pioli" << endl;
	for (vector<PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){

		PointCloudPtr cloud_cluster (new PointCloud<pcl::PointXYZRGB>);
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_clean->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		if (ExtractObject::isPioloSx(cloud_cluster) || ExtractObject::isPioloDx(cloud_cluster)){
			Utils::setColor(cloud_cluster, 0, 255, 0);
			Utils::copyTo(cloud_cluster, pioli);
		}/*else{
			Utils::setColor(cloud_cluster, 255, 255, 0);
			Utils::copyTo(cloud_cluster, pioli);
		}*/
	}

	//********************
//	PointXYZRGB p;
//	p.x = SX_MIN_X; p.y = SX_MIN_Y; p.z = SX_MIN_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = SX_MAX_X; p.y = SX_MIN_Y; p.z = SX_MIN_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = SX_MAX_X; p.y = SX_MAX_Y; p.z = SX_MIN_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = SX_MIN_X; p.y = SX_MAX_Y; p.z = SX_MIN_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = SX_MIN_X; p.y = SX_MIN_Y; p.z = SX_MAX_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = SX_MAX_X; p.y = SX_MIN_Y; p.z = SX_MAX_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = SX_MAX_X; p.y = SX_MAX_Y; p.z = SX_MAX_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = SX_MIN_X; p.y = SX_MAX_Y; p.z = SX_MAX_Z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//
//	//dx
//	p.x = DX_MIN_X; p.y = DX_MIN_Y; p.z = DX_MIN_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = DX_MAX_X; p.y = DX_MIN_Y; p.z = DX_MIN_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = DX_MAX_X; p.y = DX_MAX_Y; p.z = DX_MIN_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = DX_MIN_X; p.y = DX_MAX_Y; p.z = DX_MIN_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = DX_MIN_X; p.y = DX_MIN_Y; p.z = DX_MAX_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = DX_MAX_X; p.y = DX_MIN_Y; p.z = DX_MAX_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = DX_MAX_X; p.y = DX_MAX_Y; p.z = DX_MAX_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = DX_MIN_X; p.y = DX_MAX_Y; p.z = DX_MAX_Z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);

	//********************

	return pioli;
}


PointCloudPtr ExtractObject::extractCable(const PointCloudPtr& cloud){

	PointCloud<PointXYZRGB>::Ptr cloud_clean (new PointCloud<PointXYZRGB>);
	ConditionAnd<PointXYZRGB>::Ptr range_cond (new ConditionAnd<PointXYZRGB> ());

	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("x", ComparisonOps::GE, -37)));
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("x", ComparisonOps::LE, 10)));

	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
				(new FieldComparison<PointXYZRGB> ("y", ComparisonOps::LE, -35)));

	ConditionalRemoval<PointXYZRGB> condrem (range_cond);
	condrem.setInputCloud (cloud);
	condrem.filter (*cloud_clean);


	Utils::setColor(cloud_clean, 0, 0, 255);
	CloudIO::visualize(cloud, "cloud", cloud_clean, "zona interesse cavo");

	cout << "Estraggo cavo" << endl;
	PointCloudPtr cable(new PointCloud<PointXYZRGB>);


	cout << "Clusterizzazione" << endl;
	vector<PointIndices> indices = ExtractObject::clusterization(cloud_clean, 0.7, 700, 10000);

	cout << "Cerco cluster cavo" << endl;
	for (vector<PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){
		PointCloudPtr cloud_cluster (new PointCloud<pcl::PointXYZRGB>);

		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			cloud_cluster->points.push_back (cloud_clean->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		if (ExtractObject::isCable(cloud_cluster)){
			Utils::setColor(cloud_cluster, 255, 0, 0);
			Utils::copyTo(cloud_cluster, cable);
		}/*else{
			Utils::setColor(cloud_cluster, 255*rand(), 255*rand(), 255*rand());
			Utils::copyTo(cloud_cluster, cable);
		}*/
	}


	//********************
//	PointXYZRGB p;
//	p.x = CABLE_MIN_X; p.y = CABLE_MIN_Y; p.z = CABLE_MIN_Z;
//	p.r = 0; p.g = 0; p.b = 255; cable->push_back(p);
//
//	p.x = CABLE_MAX_X; p.y = CABLE_MIN_Y; p.z = CABLE_MIN_Z;
//	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
//
//	p.x = CABLE_MAX_X; p.y = CABLE_MAX_Y; p.z = CABLE_MIN_Z;
//	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
//
//	p.x = CABLE_MIN_X; p.y = CABLE_MAX_Y; p.z = CABLE_MIN_Z;
//	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
//
//	p.x = CABLE_MIN_X; p.y = CABLE_MIN_Y; p.z = CABLE_MAX_Z;
//	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
//
//	p.x = CABLE_MAX_X; p.y = CABLE_MIN_Y; p.z = CABLE_MAX_Z;
//	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
//
//	p.x = CABLE_MAX_X; p.y = CABLE_MAX_Y; p.z = CABLE_MAX_Z;
//	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
//
//	p.x = CABLE_MIN_X; p.y = CABLE_MAX_Y; p.z = CABLE_MAX_Z;
//	p.r = 255; p.g = 0; p.b = 255; cable->push_back(p);
	/*****************/

	return cable;
}


bool ExtractObject::isCable(const PointCloudPtr& cluster){

	int inlier = 0;
	for (int i = 0; i <cluster->width; i++){
		PointXYZRGB p;
		p.x = cluster->at(i).x;
		p.y = cluster->at(i).y;
		p.z = cluster->at(i).z;

		if (p.x > CABLE_MIN_X && p.x < CABLE_MAX_X
				&& p.y > CABLE_MIN_Y && p.y < CABLE_MAX_Y
				&& p.z > CABLE_MIN_Z && p.z < CABLE_MAX_Z){
			inlier++;
		}
	}
	cout << "cable inlier: " << inlier << " su " << cluster->width << " " << (float)inlier/cluster->points.size() << endl;
	return (float)inlier/cluster->width > 0.7;

}


bool ExtractObject::isPioloSx(const PointCloudPtr& cloud){

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


bool ExtractObject::isPioloDx(const PointCloudPtr& cloud){

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


vector<PointIndices> ExtractObject::clusterization(const PointCloudPtr& cloud, double tolerance,
												   double min, double max){

	vector<PointIndices> indices;
    search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
    tree->setInputCloud (cloud);

    EuclideanClusterExtraction<PointXYZRGB> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min);
    ec.setMaxClusterSize (max);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (indices);

    return indices;
}


bool ExtractObject::isCableCorrect(const PointCloudPtr& pioli, const PointCloudPtr& cable){
	double maxX, maxY, maxZ;
	double minX, minY, minZ;

	maxX = maxY = maxZ = -INFINITY;
	minX = minY = minZ = +INFINITY;
	maxZ = 0;

	//calcola le coordinate entro le quali devono passare i punti
	//del cavo per considerarli tra i due pioli
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

		if (p.z < minZ)
			minZ = p.z;
	}

	//conta quanti punti passano fra i due pioli
	int inliers = 0;
	for(int i = 0; i < cable->points.size(); i++){
		PointXYZRGB p = cable->points[i];
		if (p.x > minX && p.x < maxX
				&& p.y > minY && p.y < maxY
				&& p.z > minZ && p.z < maxZ){
			inliers++;
		}
	}

	cout << "Punti passanti per i pioli: " << inliers << " su " << cable->points.size() << endl;
	return inliers > 60;
}
