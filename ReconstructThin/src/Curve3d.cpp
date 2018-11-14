#include "Curve3d.h"

Curve3d::Curve3d(Eigen::Vector3d pt_begin, Eigen::Vector3d pt_end, int num_points) {
  num_points_ = num_points;
  auto step_vector = (pt_end - pt_begin) / static_cast<double>(num_points - 1);
  auto iter_vector = pt_begin;
  for (int i = 0; i < num_points; i++) {
    points_.push_back(iter_vector);
    iter_vector += step_vector;
  }
}