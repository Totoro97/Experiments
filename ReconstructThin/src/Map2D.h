#pragma once
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
// Eigen
#include <eigen3/Eigen/Eigen>

// std
#include <string>

class Map2D {
public:
  int height_, width_;
  double *map2d_;
  Eigen::Matrix3d K_, R_;
  Eigen::Vector3d T_;

  Map2D(const cv::Mat &img, bool calc_distmap = true, std::string distmap_path = "");
  ~Map2D();

  void SetKRT(Eigen::Matrix3d K, Eigen::Matrix3d R, Eigen::Vector3d T);
  double MinDist2Edge(Eigen::Vector3d pt) const;

  void CalcDistmap(const cv::Mat& img, std::string distmap_path);
  void LoadDistmap(std::string distmap_path);
};