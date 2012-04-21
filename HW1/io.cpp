/**
 * Funzioni per l'IO
 */

#include "io.h"

using namespace std;
using namespace cv;

void loadPhotoName(std::list<std::string>& list, const std::string& filename);

PhotoIO::PhotoIO(){
	this->path = "calibration_set/";
	loadPhotoName(this->sx, "input_photo_sx.txt");
	loadPhotoName(this->dx, "input_photo_dx.txt");
}

bool PhotoIO::isEmpty(){
	return (this->sx.size() == 0 || this->dx.size() == 0);
}

void PhotoIO::getNext(cv::Mat& sx, cv::Mat& dx){
	if (this->isEmpty()){
		cout << "PhotoIO::getNext: le liste sono vuote!" << endl;
		return;
	}

	sx = imread(this->path+this->sx.front(), CV_8UC1);
	dx = imread(this->path+this->dx.front(), CV_8UC1);

	cout << this->path+this->sx.front() << "  " << this->path+this->dx.front() << endl;

	this->sx.pop_front();
	this->dx.pop_front();
}

void loadPhotoName(std::list<std::string>& list, const std::string& filename){
	string line;
	ifstream iFile(filename.c_str());

	if (iFile.is_open()){
	    while ( iFile.good() ){
	      getline (iFile,line);
	      cout << line << endl;
	      if (line.size() > 1)
	    	  list.push_back(line);
	    }
	    iFile.close();
	}else
		cout << "Unable to open file " << filename << endl;
}

list<string> getFilenameList(string dirName, CameraType type){

	list<string> list;
	DIR *dir = opendir(dirName.c_str());
	struct dirent *file;
	string startName;
	string extension = string(".ppm");

	if (type == RIGHT_CAMERA)
		startName = string("right");
	else
		startName = string("left");


	while ((file = readdir(dir)) != NULL){
		if (endWith(file->d_name, extension.c_str()) && startWith(file->d_name, startName.c_str())){
//			cout << file->d_name << endl; //todo debug
			std::list<string>::iterator it;
			for (it = list.begin(); it != list.end() && it->compare(file->d_name) < 0; it++){
//				cout << *it << " ";
			}
//			cout << endl;

			list.insert(it, file->d_name);
		}
	}
	closedir(dir);


	return list;
}

bool endWith(const char *str, const char *cmp){
	if (strlen(cmp) > strlen(str))
		return false;

	int cmpLen = strlen(cmp);
	int strLen = strlen(str);

	for (int i = 0; i < cmpLen; i++)
		if (str[strLen-1-i] != cmp[cmpLen-1-i])
			return false;

	return true;
}

bool startWith(const char *str, const char *cmp){
	if (strlen(cmp) > strlen(str))
		return false;

	int cmpLen = strlen(cmp);

	for (int i = 0; i < cmpLen; i++)
		if (str[i] != cmp[i])
			return false;

	return true;
}

void writeMatFile(const string& filename, const string& matname, const cv::Mat& mat){
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(fs.isOpened())
    {
        fs << matname <<  mat;
        fs.release();
    }
    else
        cout << "Errore: impossibile salvare " << matname << " in " << filename << "\n";
}
