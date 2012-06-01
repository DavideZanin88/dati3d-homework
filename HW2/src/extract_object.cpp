
#include "extract_object.h"


using namespace std;
using namespace pcl;


const PointXYZ ExtractObject::PSX_MIN(-27.5, -43.5, -9.5);
const PointXYZ ExtractObject::PSX_MAX(-21.5, -37, -7.5);

const PointXYZ ExtractObject::PDX_MIN(-22, -43.5, -8.5);
const PointXYZ ExtractObject::PDX_MAX(-17, -37, -6.5);

const PointXYZ ExtractObject::CABLE_MIN(-35, -60, -15);
const PointXYZ ExtractObject::CABLE_MAX(0, -30, -2);


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

//	ExtractObject::setColor(cloud_clean, 0, 0, 255);
//	CloudIO::visualize(cloud, "cloud", cloud_clean, "zona pioli");

	cout << "Clusterizzazione" << endl;
	vector<PointIndices> indices = ExtractObject::clusterization(cloud_clean, 0.6, 3, 75);

	cout << "Cerco i cluster dei pioli" << endl;
	for (vector<PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){

		PointCloudPtr cloud_cluster (new PointCloud<pcl::PointXYZRGB>);
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_clean->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		if (ExtractObject::isIn(cloud_cluster, PSX_MIN, PSX_MAX)
				|| ExtractObject::isIn(cloud_cluster, PDX_MIN, PDX_MAX)){
			ExtractObject::setColor(cloud_cluster, 0, 255, 0);
			ExtractObject::copyTo(cloud_cluster, pioli);
		}/*else{
			ExtractObject::setColor(cloud_cluster, 255, 255, 0);
			ExtractObject::copyTo(cloud_cluster, pioli);
		}*/
	}

	//********************
//	PointXYZRGB p;
//	p.x = PSX_MIN.x ; p.y = PSX_MIN.y; p.z = PSX_MIN.z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//	p.x = PSX_MAX.x ; p.y = PSX_MAX.y; p.z = PSX_MAX.z;
//	p.r = 0; p.g = 0; p.b = 255; pioli->push_back(p);
//
//
//	p.x = PDX_MIN.x ; p.y = PDX_MIN.y; p.z = PDX_MIN.z;
//	p.r = 0; p.g = 255; p.b = 255; pioli->push_back(p);
//
//	p.x = PDX_MAX.x ; p.y = PDX_MAX.y; p.z = PDX_MAX.z;
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


//	ExtractObject::setColor(cloud_clean, 0, 0, 255);
//	CloudIO::visualize(cloud, "cloud", cloud_clean, "zona interesse cavo");

	cout << "Estraggo cavo" << endl;
	PointCloudPtr cable(new PointCloud<PointXYZRGB>);


	cout << "Clusterizzazione" << endl;
	vector<PointIndices> indices = ExtractObject::clusterization(cloud_clean, 0.7, 700, 3000);

	cout << "Cerco cluster cavo" << endl;
	for (vector<PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){
		PointCloudPtr cluster (new PointCloud<pcl::PointXYZRGB>);

		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			cluster->points.push_back (cloud_clean->points[*pit]);
		}
		cluster->width = cluster->points.size ();
		cluster->height = 1;
		cluster->is_dense = true;

		if (ExtractObject::isIn(cluster, CABLE_MIN, CABLE_MAX)){
			ExtractObject::setColor(cluster, 255, 0, 0);
			ExtractObject::copyTo(cluster, cable);
		}/*else{
			ExtractObject::setColor(cloud_cluster, 255*rand(), 255*rand(), 255*rand());
			ExtractObject::copyTo(cloud_cluster, cable);
		}*/
	}


	//********************
//		PointXYZRGB p;
//		p.x = CABLE_MIN.x ; p.y = CABLE_MIN.y; p.z = CABLE_MIN.z;
//		p.r = 0; p.g = 0; p.b = 255; cable->push_back(p);
//
//		p.x = CABLE_MAX.x ; p.y = CABLE_MAX.y; p.z = CABLE_MAX.z;
//		p.r = 0; p.g = 0; p.b = 255; cable->push_back(p);
	//******************/

	return cable;
}


bool ExtractObject::isIn(const PointCloudPtr& cloud, PointXYZ min, PointXYZ max){
	int inlier = 0;
	for (int i = 0; i < cloud->points.size(); i++){
		PointXYZRGB p = cloud->points[i];
		if (p.x > min.x && p.x < max.x &&
			p.y > min.y && p.y < max.y &&
			p.z > min.z && p.z < max.z){
			inlier++;
		}
	}
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
	return inliers > 55;
}


void ExtractObject::setColor(const PointCloudPtr& cloud, char r, char g, char b){
	for (int i = 0; i < cloud->points.size(); i++){
		cloud->points[i].r = r;
		cloud->points[i].g = g;
		cloud->points[i].b = b;
	}
}


void ExtractObject::copyTo(const PointCloudPtr& src, const PointCloudPtr& dest){
	for (int i = 0; i < src->points.size(); i++){
		dest->points.push_back(src->points[i]);
	}
	dest->width = dest->points.size ();
	dest->height = 1;
	dest->is_dense = true;
}

