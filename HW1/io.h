
#ifndef IO_H_
#define IO_H_

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <fstream>
#include <list>
#include <string>

#include <dirent.h>
#include <stdio.h>
#include <string.h>

#include <list>
#include <string>

enum CameraType{LEFT_CAMERA, RIGHT_CAMERA};

class PhotoIO{

	public:
		PhotoIO();

		bool isEmpty();
		void getNext(cv::Mat& sx, cv::Mat& dx);

	private:
		std::list<std::string> sx;
		std::list<std::string> dx;
		std::string path;

};

std::list<std::string> getFilenameList(std::string dirName, CameraType type);
void writeMatFile(const std::string& filename, const std::string& matname, const cv::Mat& mtat);

bool endWith(const char *str, const char *cmp);
bool startWith(const char *str, const char *cmp);


#endif /* IO_H_ */
