
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>


#include "cloud_io.h"
#include "ref_cloud.h"



using namespace std;
using namespace pcl;

int main(int argc, char **argv){

	const string REF_CLOUD_PATH = "dataset/correct02/inliers.pcd";

	cout << "Avvio hw2..." << endl;

//	pcl::console::setVerbosityLevel(console::L_VERBOSE);


	if (argc < 3){
		cout << "Specificare la cloud di input!" << endl;
		return -1;
	}

	PointCloudPtr cable02 = CloudIO::loadPointCloud(argv[1]);


	//carica cable
//	PointCloudPtr ref = CloudIO::loadPointCloud("dataset/correct02/inliers.pcd", true);
	RefCloud refCloud(REF_CLOUD_PATH);

	PointCloud<PointXYZRGB>::Ptr reg;
	if (strcmp(argv[2], "true") == 0){
		cout << "Avvia registrazione" << endl;
		reg = refCloud.registration(cable02);
	}else
		reg = cable02;

	PointCloudPtr pioli = ExtractObject::extractPioli2(reg);
	CloudIO::visualize(reg, "tutto", pioli, "pioli");


	PointCloudPtr cable = ExtractObject::extractCable(reg);

	if (ExtractObject::isCableCorrect(pioli, cable)){
		cout << "SI" << endl;
	}else{
		cout << "NO" << endl;
	}

//	CloudIO::visualize(cable, "cable");

	//visualizzo una point cloud
	CloudIO::visualize(reg, "ref", pioli, "pioli", cable, "cable");



	cout << "Fine hw2..." << endl;
}



