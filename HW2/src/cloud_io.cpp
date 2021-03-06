
#include "cloud_io.h"

using namespace std;
using namespace pcl;



PointCloudPtr CloudIO::loadPointCloud(const string& path){

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	//carica la point cloud
	if (io::loadPCDFile<pcl::PointXYZRGB> (path, *cloud) == -1){
		PCL_ERROR ("Couldn't read the pcd file \n");
		return cloud;
	}
	cloud->width = cloud->size();
	cloud->height = 1;

	//sposta in modo che il centroide si trovi in (0,0,0)
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud, centroid);
	PointCloud<PointXYZRGB>::Ptr cloud_xyz_demean (new PointCloud<PointXYZRGB>);
	demeanPointCloud<PointXYZRGB> (*cloud, centroid, *cloud_xyz_demean);
	cloud = cloud_xyz_demean;

	//rimuove il tavolo
	PointCloudPtr clean (new PointCloud<PointXYZRGB>);
	ConditionAnd<PointXYZRGB>::Ptr range_cond (new ConditionAnd<PointXYZRGB> ());
	range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr
			(new FieldComparison<PointXYZRGB> ("z", ComparisonOps::LE, 0)));
	ConditionalRemoval<PointXYZRGB> condrem (range_cond);
	condrem.setInputCloud (cloud);
	condrem.filter (*clean);

	return clean;
}

//callback che stampa le coordinate del punto cliccato
void pp_callback(const visualization::PointPickingEvent& event, void* args);

void CloudIO::visualize(const PointCloudPtr& cloud, const string& name){

	visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(0.1);
	viewer.addText(name, 15, 15);

	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
	viewer.addPointCloud<PointXYZRGB> (cloud, rgb, name);
	viewer.registerPointPickingCallback (pp_callback, (void *)&cloud);

	cout << "Visualization: "<< name << endl;
	while (!viewer.wasStopped ()){
		viewer.spin();
	}

}

void CloudIO::visualize(const PointCloudPtr& cloud1, const string& name1,
						const PointCloudPtr& cloud2, const string& name2){


	visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(0.1);
	viewer.addText(name1 + "," + name2, 15, 15);

	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb1(cloud1);
	viewer.addPointCloud<PointXYZRGB> (cloud1, rgb1, name1);

	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb2(cloud2);
	viewer.addPointCloud<PointXYZRGB> (cloud2, rgb2, name2);

	cout << "Visualization: "<< name1 << ", " << name2 << endl;
	while (!viewer.wasStopped ()){
		viewer.spin();
	}

}


void CloudIO::visualize(const PointCloudPtr& cloud1, const string& name1,
						const PointCloudPtr& cloud2, const string& name2,
						const PointCloudPtr& cloud3, const string& name3){


	visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(0.1);
	viewer.addText(name1 + "," + name2 + "," + name3, 15, 15);

	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb1(cloud1);
	viewer.addPointCloud<PointXYZRGB> (cloud1, rgb1, name1);

	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb2(cloud2);
	viewer.addPointCloud<PointXYZRGB> (cloud2, rgb2, name2);

	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb3(cloud3);
		viewer.addPointCloud<PointXYZRGB> (cloud3, rgb3, name3);

	cout << "Visualization: "<< name1 << ", " << name2 << ", " << name3 << endl;
	while (!viewer.wasStopped ()){
		viewer.spin();
	}

}


void pp_callback (const visualization::PointPickingEvent& event, void* args){
	PointCloud<PointXYZRGB>::Ptr cloud = *(PointCloud<PointXYZRGB>::Ptr *)args;
	int index = event.getPointIndex ();
	if (index == -1)
		return;

	cout << cloud->points[index].x << " " << cloud->points[index].y << " " << cloud->points[index].z << endl;
}





