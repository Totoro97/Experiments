CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
PROJECT(FitCameraPose)
SET(CMAKE_CXX_STANDARD 14)
# SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "/O2")
SET(CMAKE_BUILD_TYPE release)

# OpenCV
SET(OpenCV_DIR C:/Users/Aska/Software/OpenCV/opencv/build/x64/vc15/lib)
FIND_PACKAGE(OpenCV)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(C:/Users/Aska/Software/OpenCV/opencv/build/x64/vc15/lib)
LINK_DIRECTORIES(C:/Users/Aska/Software/OpenCV/opencv/build/x64/vc15/bin)

# Eigen
INCLUDE_DIRECTORIES(C:/Users/Aska/Software/Eigen/eigen/Eigen)

SET(SOURCE_FILES Headers.h)
SET(SOURCE_FILES ${SOURCE_FILES} View.h View.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} Utils.h Utils.cpp)

#ADD_EXECUTABLE(FitCameraPose main.cpp ${SOURCE_FILES})
#TARGET_LINK_LIBRARIES(FitCameraPose ${OpenCV_LIBS})

ADD_EXECUTABLE(ChangeFormat ChangeFormat.cpp ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(ChangeFormat ${OpenCV_LIBS})