#pragma once
#include "Headers.h"
#include <string>

class View {
public:
  View(std::string image_path, bool calc_dist_map = false, std::string map_path = ".");
  ~View();
  void SetKRT(Eigen::Matrix3d K, Eigen::Matrix3d R, Eigen::Vector3d T);
  void CalcDistMap(const cv::Mat &img, std::string map_path);
  void LoadDistMap(std::string map_path);
  double Distance(Eigen::Vector3d pt);
  bool IsInside(int i, int j);
  Eigen::Vector2d Map3to2(Eigen::Vector3d pt);
  Eigen::Vector3d Map2to3(double a, double b, double depth);
  void SamplePoints(std::vector<Eigen::Vector3d> &points, double weight);

  int height_;
  int width_;
  double *dist_map_;
  cv::Mat img_;
  Eigen::Matrix3d K_, R_;
  Eigen::Vector3d T_;
  Eigen::Matrix4d view_to_world_;
};