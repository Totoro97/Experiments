cmake_minimum_required(VERSION 3.13)
project(OctreeCalibration)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

#OpenCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIR})

#Eigen
include_directories(/usr/include/eigen3/Eigen)
