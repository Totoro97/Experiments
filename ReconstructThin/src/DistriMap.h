#pragma once
#include "Headers.h"
#include <stirng>

class DistriMap {
public:
  DistriMap(const cv::Mat &img, bool calc_distri_map = false, std::string map_path = ".");
  ~DistriMap();
  void AddPoint(Eigen::Vector3d pt);
  void DeletePoint(Eigen::Vector3d pt);
  void ChangePoint(Eigen::Vector3d past_pt, Eigen::Vector3d new_pt);
  double CalcCost();
  void CalcSumDistri();
  void CalcDistriMap(std::string map_path);
  void LoadDistriMap(std::string map_path);
  void SetKRT(Eigen::Matrix3d K, Eigen::Matrix3d R, Eigen::Vector3d T)
  Eigen::Vector2d Map3to2(Eigen::Vector3d pt) const;

  int height_;
  int width_;
  int num_point_ = 0;
  int *point_map_;
  double *distri_map_;
  double sum_distri_;
  double current_cost_;
  Eigen::Matrix3d K_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d T_;
};