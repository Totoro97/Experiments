#include "DistMap.h"
#include "Headers.h"
#include "Utils.h"
// std
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
  int num_frame = 64;
  std::vector<DistMap *> dist_maps;
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < num_frame; i++) {
    auto img =
      cv::imread(std::string("/home/totoro/tmp/") + std::to_string(i) + std::string(".png"));
    auto tmp_ptr = new DistMap(img, false, std::to_string(i) + std::string("bin"));
    dist_maps.push_back(tmp_ptr);
  }
  const double pi = std::acos(-1.0);
  double ang_step = pi * 2.0 / static_cast<double>(num_frame);
  int cnt = 0;
  double sum_dist_thres = 50;
  for (double alpha = 0; alpha < pi * 2.0; alpha += 0.01) {
    for (double r = 0.01; r < 0.75; r += 0.01) {
      for (double h = -1.0; h < 1.0; h += 0.01) {
        //points.emplace_back(
        //  std::cos(alpha) / (1.0 / r + std::sin(alpha)), h / (1.0 + r * std::sin(alpha)), 1.0);
        //continue;
        if ((cnt++) % 10000 == 0) {
          std::cout << "Current: alpha: " << alpha << " r: " << r << " h: " << h << std::endl;
        }
        double current_ang = alpha;
        double sum_dist = 0.0;
        for (int i = 0; i < num_frame; i++) {
          double a = std::cos(current_ang) / (1.0 / r + std::sin(current_ang)) * 1050.0;
          double b = -h / (1.0 + r * std::sin(current_ang)) * 1050.0;
          sum_dist += dist_maps[i]->Distance(a, b);
          current_ang -= ang_step;
        }
        if (sum_dist < sum_dist_thres) {
          //double a = std::cos(alpha) / (1.0 / r + std::sin(alpha)) * 105.0;
          //double b = -h / (1.0 + r * std::sin(alpha)) * 105.0;
          Eigen::Vector3d tmp(r * std::cos(alpha), 1.0 + r * std::sin(alpha), h);
          points.push_back(tmp * 10.0);
          //std::cout << "yes! = " << a << " " << b << std::endl;
        }
      }
    }
  }

  for (int t = 0; t < num_frame; t++) {
    auto distri_ptr = dist_maps[t];
    auto tmp_ptr = new double[distri_ptr->width_ * distri_ptr->height_];
    for (int i = 0; i < distri_ptr->width_ * distri_ptr->height_; i++) {
      tmp_ptr[i] = std::min(distri_ptr->dist_map_[i], 50.0);
    }
    Utils::SaveGrayScaleImageFromPtr(
      tmp_ptr,
      distri_ptr->width_,
      distri_ptr->height_,
      std::to_string(t) + std::string("just.png")
    );
    delete(tmp_ptr);
  }
  std::cout << "num_points: " << points.size() << std::endl;
  Utils::SavePoints(std::string("points.json"), points);
  return 0;
}