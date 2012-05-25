
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

	//	pcl::console::setVerbosityLevel(console::L_VERBOSE);

	const string REF_CLOUD_PATH = "dataset/correct02/inliers.pcd";
	bool result[100];

	if (argc < 2){
		cout << "Specificare almeno una point cloud di input!" << endl;
		return -1;
	}
	if (argc > 100){
		cout << "Verranno analizzate solo le prime 100 point cloud" << endl;
	}

	//carico la point cloud di riferimento
	RefCloud refCloud(REF_CLOUD_PATH);

	//analizzo le cloud passate come parametro
	for (int i = 1; i < argc; i++){
		cout << "Analizzo la point cloud: " << argv[i] << endl;
		PointCloudPtr cloud = CloudIO::loadPointCloud(argv[i]);
		cloud = refCloud.registration(cloud);
		CloudIO::visualize(cloud, "registrata");

		PointCloudPtr pioli = ExtractObject::extractPioli(cloud);
		CloudIO::visualize(cloud, "cloud", pioli, "pioli");

		PointCloudPtr cable = ExtractObject::extractCable(cloud);

		if ((result[i-1] = ExtractObject::isCableCorrect(pioli, cable))){
			cout << "Risultato: SI" << endl;
		}else{
			cout << "Risultato: NO" << endl;
		}

		CloudIO::visualize(cloud, "ref", pioli, "pioli", cable, "cable");
	}

	cout << "Riepilogo dei risultati" << endl;
	for (int i = 1; i < argc; i++){
		cout << argv[i] << " ";
		cout << (result[i-1] ? "SI": "NO") << endl;
	}

//	PointCloudPtr cable02 = CloudIO::loadPointCloud(argv[1]);
//
//
//	//carica cable
////	PointCloudPtr ref = CloudIO::loadPointCloud("dataset/correct02/inliers.pcd", true);
//
//
//	PointCloud<PointXYZRGB>::Ptr reg;
//	if (strcmp(argv[2], "true") == 0){
//		cout << "Avvia registrazione" << endl;
//		reg = refCloud.registration(cable02);
//	}else
//		reg = cable02;
//
//	PointCloudPtr pioli = ExtractObject::extractPioli2(reg);
//	CloudIO::visualize(reg, "tutto", pioli, "pioli");
//
//
//	PointCloudPtr cable = ExtractObject::extractCable(reg);
//
//	if (ExtractObject::isCableCorrect(pioli, cable)){
//		cout << "SI" << endl;
//	}else{
//		cout << "NO" << endl;
//	}
//
////	CloudIO::visualize(cable, "cable");
//
//	//visualizzo una point cloud
//	CloudIO::visualize(reg, "ref", pioli, "pioli", cable, "cable");



	cout << "Fine hw2..." << endl;
}



