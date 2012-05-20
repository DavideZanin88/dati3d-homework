
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>


#include "mypointcloud.h"
#include "cloud_io.h"
#include "my_datatype.h"
#include "extract_object.h"


using namespace std;
using namespace pcl;

int main(int argc, char **argv){

	cout << "Avvio hw2..." << endl;

//	pcl::console::setVerbosityLevel(console::L_VERBOSE);


//	PointCloudPtr cable02 = CloudIO::loadPointCloud("dataset/cable02/inliers.pcd");


	//carica cable
	PointCloudPtr ref = CloudIO::loadPointCloud("dataset/correct02/inliers.pcd");

	cout << "Avvia registrazione" << endl;
//	PointCloud<PointXYZRGB>::Ptr reg = MyPointCloud::pclRegister(cable02, ref);
	PointCloudPtr reg = ref; //TODO deve essere quella registrata!
//	CloudIO::visualize(reg, "registrata");


	PointCloudPtr pioli = ExtractObject::extractPioli(reg);
	PointCloudPtr cable = ExtractObject::extractCable(reg);

	ExtractObject::isCableCorrect(pioli, cable);

//	CloudIO::visualize(cable, "cable");

	//visualizzo una point cloud
	CloudIO::visualize(reg, "ref", pioli, "pioli", cable, "cable");



	cout << "Fine hw2..." << endl;
}



