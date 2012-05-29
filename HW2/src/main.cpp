
#include "cloud_io.h"
#include "ref_cloud.h"
#include "extract_object.h"


using namespace std;
using namespace pcl;


int main(int argc, char **argv){

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
		cout << "------------" << endl;
		cout << "Analizzo la point cloud: " << argv[i] << endl;
		PointCloudPtr cloud = CloudIO::loadPointCloud(argv[i]);
		cloud = refCloud.registration(cloud);
//		CloudIO::visualize(cloud, "registrata");

		PointCloudPtr pioli = ExtractObject::extractPioli(cloud);
//		CloudIO::visualize(cloud, "cloud", pioli, "pioli");

		PointCloudPtr cable = ExtractObject::extractCable(cloud);

		if ((result[i-1] = ExtractObject::isCableCorrect(pioli, cable))){
			cout << "Risultato: SI" << endl;
		}else{
			cout << "Risultato: NO" << endl;
		}

		CloudIO::visualize(cloud, "ref", pioli, "pioli", cable, "cable");
	}

	cout << endl << "Riepilogo dei risultati:" << endl;
	for (int i = 1; i < argc; i++){
		cout << argv[i] << " ";
		cout << (result[i-1] ? "SI": "NO") << endl;
	}

}



