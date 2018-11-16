#pragma once
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
// Eigen
#include <eigen3/Eigen/Eigen>

// std

class Map2D {
public:
  int height_, width_;
  double *map2d_;
  Eigen::Matrix3d K_, R_;
  Eigen::Vector3d T_;

  Map2D(const cv::Mat &img);
  ~Map2D();

  double MinDist2Edge(Eigen::Vector3d pt) const;
};