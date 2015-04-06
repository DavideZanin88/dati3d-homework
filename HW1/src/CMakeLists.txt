
cmake_minimum_required(VERSION 2.6) 

project(HW1)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

set(Boost_REALPATH ON)

include_directories(${OPENCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ../include)
link_directories(${OPENCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS} ${PCL_DEFINITIONS})


add_executable (hw1 main.cpp calibration.cpp disparity.cpp io.cpp mypointcloud.cpp)
target_link_libraries(hw1 ${OpenCV_LIBS} ${PCL_LIBRARIES})


