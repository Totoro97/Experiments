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

  int cnt = 0;
  for (auto curve = curves_.begin(); curve != curves_.end(); curve++) {
    std::cout << "Now iter: " << ++cnt << std::endl;
    for (int iter_num = 100; iter_num > 0; iter_num--) {
      for (auto pt = curve->points_.begin(); pt != curve->points_.end(); pt++) {
        double current_dis = 0.0;
        //std::cout << "pt = " << pt << std::endl;
        for (const auto &map2d : *map2ds_) {
          double single_dis = map2d->MinDist2Edge(*pt);
          current_dis += single_dis;
        }
        //std::cout << "current_dis = " << current_dis << std::endl;
        //if (current_dis > 1e7) {
        //  continue;
        //}
        Eigen::Vector3d grad(0.0, 0.0, 0.0);
        for (int index = 0; index < 3; index++) {
          double new_dis = 0.0;
          const double step_length = 1e-2;
          Eigen::Vector3d bias(0.0, 0.0, 0.0);
          bias(index) = step_length;
          for (const auto &map2d: *map2ds_) {
            double single_dis = map2d->MinDist2Edge(*pt + bias);
            new_dis += single_dis;  
          }
          //std::cout << "new_dis = " << new_dis << std::endl;
          //if (new_dis > 1e7) {
          //  valid = false;
          //  break;
          //}
          grad(index) = (new_dis - current_dis) / step_length;
        }
        const double step_ratio = 1e-5;
        *pt = *pt - grad * step_ratio;
      }
    }
  }
  return;
}