#include "Bend.h"

Bend::Bend() {
}

void Bend::GoBendNow() {
  line_generator_->GetLine3D(lines_);
  const num_points = 10;
  for (const auto &line : lines_) {
    curves_.emplace_back(line.first, line.second, num_points);   
  }

  for (auto &curve : curves_) {
    double past_cost = 1e15;
    for (iter_num = 100; iter_num > 0; iter_num--) {
      for (auto &pt : curve.points_) {
        double current_dis = 0.0;
        for (const auto &map2d : map2ds_) {
          double single_dis = map2d.MinDist2Edge(pt);  
          current_dis += single_dis;
        }
        Eigen::Vector3d grad(0.0, 0.0, 0.0);
        for (int index = 0; index < 3; index++) {
          double new_dis = 0.0;
          const double step_length = 1e-3;
          Eigen::Vector3d bias(0.0, 0.0, 0.0);
          bias(i) = step_length;
          for (const auto &map2d: map2ds_) {
            double single_dis = map2d.MinDist2Edge(pt + bias);
            new_dis += single_dis;  
          }
          grad(i) = (new_dis - current_dis) / step_length;
        }
        const double step_ratio = 1.0;
        pt = pt + grad * step_ratio;
      }
    }
  }
  return;
}