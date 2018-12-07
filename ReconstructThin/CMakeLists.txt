CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(thin)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-O2")
SET(THIRDPARTY_DIR ${PROJECT_SOURCE_DIR}/../ThirdParty)

# Eigen
INCLUDE_DIRECTORIES(${THIRDPARTY_DIR}/Eigen)

# Edge_Draw
INCLUDE_DIRECTORIES(${THIRDPARTY_DIR}/edge-drawing/include)
LINK_DIRECTORIES(${THIRDPARTY_DIR}/edge-drawing/build)

# Line3Dpp
INCLUDE_DIRECTORIES(${THIRDPARTY_DIR}/Line3Dpp/build)
INCLUDE_DIRECTORIES(${THIRDPARTY_DIR}/Line3Dpp)
LINK_DIRECTORIES(${THIRDPARTY_DIR}/Line3Dpp/build)

# openCV
FIND_PACKAGE(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)

SET(SOURCE_FILES src/Bend.h src/Bend.cpp src/Curve3d.h src/Curve3d.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} src/LineGenerator.h src/LineGenerator.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} src/Map2D.h src/Map2D.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} src/Utils.h src/Utils.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} src/Tetra.h src/Tetra.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} src/Headers.h)
SET(SOURCE_FILES ${SOURCE_FILES} src/DistriMap.h src/DistriMap.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} src/Algorithms.h src/Algorithms.cpp)
SET(SOURCE_FILES ${SOURCE_FILES} src/DistMap.h src/DistMap.cpp)

ADD_EXECUTABLE(main src/Main.cpp ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(main ed)
TARGET_LINK_LIBRARIES(main line3Dpp boost_system boost_chrono)
TARGET_LINK_LIBRARIES(main ${OpenCV_LIBS})

ADD_EXECUTABLE(gen src/GenLine3D.cpp)
TARGET_LINK_LIBRARIES(gen line3Dpp boost_system boost_chrono)
TARGET_LINK_LIBRARIES(gen ${OpenCV_LIBS})

# ADD_EXECUTABLE(test_tetra src/TestTetra.cpp src/Utils.cpp src/Utils.h src/Tetra.cpp src/Tetra.h)

ADD_EXECUTABLE(suit_main src/SuitMain.cpp ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(suit_main ed)
TARGET_LINK_LIBRARIES(suit_main line3Dpp boost_system boost_chrono)
TARGET_LINK_LIBRARIES(suit_main ${OpenCV_LIBS})

ADD_EXECUTABLE(rotate_main src/RotateMain.cpp ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(rotate_main ed)
TARGET_LINK_LIBRARIES(rotate_main line3Dpp boost_system boost_chrono)
TARGET_LINK_LIBRARIES(rotate_main ${OpenCV_LIBS})
