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
      
      for (const auto &map2d : map2ds_) {
        for (auto &pt : curve.points_) {
          double single_dis = map2d.MinDist2Edge(pt);
          
        }
      }
    }
  }
  return;
}