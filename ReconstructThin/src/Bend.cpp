#include "Bend.h"
// std
#include <iostream>

void Bend::GoBendNow() {
  std::cout << "GoBendNow: Begin" << std::endl;
  lines_ = new std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >();
  line_generator_->GetSegment3D(lines_);
  std::cout << "lines_size: " << lines_->size() << std::endl;
  
  const int num_points = 10;
  for (const auto &line : *lines_) {
    curves_.emplace_back(line.first, line.second, num_points);   
  }

  for (auto &curve : curves_) {
    double past_cost = 1e15;
    for (int iter_num = 100; iter_num > 0; iter_num--) {
      std::cout << "Now iter: " << iter_num << std::endl;
      for (auto &pt : curve.points_) {
        double current_dis = 0.0;
        for (const auto &map2d : *map2ds_) {
          double single_dis = map2d->MinDist2Edge(pt);  
          current_dis += single_dis;
        }
        Eigen::Vector3d grad(0.0, 0.0, 0.0);
        for (int index = 0; index < 3; index++) {
          double new_dis = 0.0;
          const double step_length = 1e-3;
          Eigen::Vector3d bias(0.0, 0.0, 0.0);
          bias(index) = step_length;
          for (const auto &map2d: *map2ds_) {
            double single_dis = map2d->MinDist2Edge(pt + bias);
            new_dis += single_dis;  
          }
          grad(index) = (new_dis - current_dis) / step_length;
        }
        const double step_ratio = 1.0;
        pt = pt + grad * step_ratio;
      }
    }
  }
  return;
}