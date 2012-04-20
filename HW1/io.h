
#ifndef IO_H_
#define IO_H_


#include <list>
#include <string>

enum CameraType{LEFT_CAMERA, RIGHT_CAMERA};

std::list<std::string> getFilenameList(std::string dirName, CameraType type);

bool endWith(const char *str, const char *cmp);
bool startWith(const char *str, const char *cmp);


#endif /* IO_H_ */
