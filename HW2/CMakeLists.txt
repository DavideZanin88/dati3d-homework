

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(hw2)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (hw2 src/main.cpp src/cloud_io.cpp src/ref_cloud.cpp src/extract_object.cpp)
target_link_libraries(hw2 ${PCL_LIBRARIES})