#pragma once
// Eigen
#include <eigen3/Eigen/Eigen>
// std
#include <vector>

class Curve3d {
public:
  Curve3d(Eigen::Vector3d pt_begin, Eigen::Vector3d pt_end, int num_points);
  
  // data
  std::vector<Eigen::Vector3d> points_;
  int num_points_;
};