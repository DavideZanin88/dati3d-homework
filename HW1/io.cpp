/**
 * Funzioni per l'IO
 */

#include <iostream>
#include <list>
#include <string>

#include <dirent.h>
#include <stdio.h>
#include <string.h>

#include "io.h"

using namespace std;

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
