#include "Algorithms.h"
#include "Utils.h"
// std
#include <algorithm>
#include <cmath>

// still working
/*
void Algo::SuitWithTube(const cv::Mat &img, std::vector<Tube> &tubes, int num_tube) {
  tubes.clear();
  while ((num_tube--) > 0) {
    Eigen::Vector2d o(Utils::RandomLR(0, img.rows - 1), Utils::RandomLR(0, img.cols - 1));
    Eigen::Vector2d v;
    do {
      v = Eigen::Vector2d(Utils::RandomLR(-1.0, 1.0), Utils::RandomLR(-1.0, 1.0));
    }
    while (v.norm() < 1e-2);
    double r = 1.0;
    double step_length = 1.0;
    auto CalcCost = [&img](Eigen::Vector2d o, Eigen::Vector2d v, double r){
      auto v_normed = v / v.norm();
      auto dir = Eigen::Vector2d(v_normed(1), -v_normed(0));
      double x_min = o(0) + std::min(0.0, v(0)) - std::abs(dir(0));
      double x_max = o(0) + std::max(0.0, v(0)) + std::abs(dir(0));
      double y_min = o(1) + std::min(0.0, v(1)) - std::abs(dir(1));
      double y_max = o(1) + std::max(0.0, v(1)) + std::abs(dir(1));
      
    }
    double current_cost = CalcCost(o, v, r);
    for (int iter_num = 100; iter_num > 0; iter_num--) {
      
    }
  }
  return;
}*/

void Algo::SuitWithPointCloud(std::vector<DistriMap *> &distri_maps,
                              std::vector<Eigen::Vector3d> &points,
                              int num_point,
                              int num_iter,
                              double init_range) {

  points.clear();
  const double pi = std::acos(-1.0);
  for (; num_point > 0; num_point--) {
    std::cout << "-----------------------new point, res: "
              << num_point << " --------------------" << std::endl;
    Eigen::Vector3d pt(Utils::RandomLR(-init_range, init_range),
                       Utils::RandomLR(-init_range, init_range),
                       Utils::RandomLR(-init_range, init_range));
    for (auto &distri_map : distri_maps) {
      distri_map->AddPoint(pt);
    }
    double step_length = 1.0;
    for (int iter_cnt = 0; iter_cnt < num_iter; iter_cnt++) {
      // std::cout << pt.transpose() << std::endl;
      double alpha = Utils::RandomLR(-pi, pi);
      double beta = Utils::RandomLR(-pi * 0.5, pi * 0.5);
      Eigen::Vector3d vec(std::cos(alpha) * std::cos(beta),
                          std::sin(alpha) * std::cos(beta),
                          std::sin(beta));
      vec *= step_length;
      step_length *= 0.995;
      // std::cout << "step_length = " << step_length << std::endl;
      auto new_pt = pt + vec;
      double past_cost = 0, new_cost = 0;
      for (auto &distri_map : distri_maps) {
        past_cost += distri_map->CalcCost();
        distri_map->ChangePoint(pt, new_pt);
        new_cost += distri_map->CalcCost();
      }

      // std::cout << "past: " << past_cost << " new: " << new_cost << std::endl;
      if (new_cost < past_cost) {
        pt = new_pt;
      } else {
        for (auto &distri_map : distri_maps) {
          distri_map->ChangePoint(new_pt, pt);
        }
      }
    }
    points.push_back(pt);
  }
  return;
}