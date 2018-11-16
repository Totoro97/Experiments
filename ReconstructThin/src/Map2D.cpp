#include "Map2D.h"
// edge draw
#include <ed.hpp>
// std
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <set>

Map2D::~Map2D() {
  delete(map2d_);
}

Map2D::Map2D(const cv::Mat& img) {
  height_ = img.rows;
  width_ = img.cols;
  map2d_ = new double[height_ * width_];
  std::fill_n(map2d_, height_ * width_, 1e9);
  ed::ED tmp;
  std::cout << "DetectEdges: Begin" << std::endl;
  auto edges = tmp.detectEdges(img);

  auto dis_cmp = [this](std::pair<int, int> a, std::pair<int, int> b) {
    double dist_a = map2d_[a.first * this->width_ + a.second];
    double dist_b = map2d_[b.first * this->width_ + b.second];
    if (dist_a < dist_b - 1e-8)
      return true;
    if (dist_a > dist_b + 1e-8)
      return false;
    return (a.first * this->width_ + a.second < b.first * this->width_ + b.second);
  };

  std::cout << "PreCalcDistMap: Begin" << std::endl;
  std::set<std::pair<int, int>, decltype(dis_cmp)> que(dis_cmp);
  for (const auto &edge : edges) {
    for (const auto &pix : edge) {
      map2d_[pix.y * width_ + pix.x] = 0;
      que.insert(std::make_pair(pix.y, pix.x));
    }
  }

  // dist map
  std::vector<std::pair<int, int> > biases;
  const int lim = 1;
  std::function<int(int, int)> gcd = [&gcd](int a, int b) {
    return b ? gcd(b, a % b) : a;
  };
  for (int i = -lim; i <= lim; i++)
    for (int j = -lim; j <= lim; j++) {
      if (gcd(std::abs(i), std::abs(j)) == 1 && abs(i) + abs(j) <= 1) {
        biases.push_back(std::make_pair(i, j));
      }
    }
  std::cout << "biased size = " << biases.size() << std::endl;
  while (!que.empty()) {
    auto pix = *que.begin();
    que.erase(que.begin());
    double current_dis = map2d_[pix.first * width_ + pix.second];
    for (const auto &bias : biases) {
      int r = pix.first + bias.first;
      int c = pix.second + bias.second;
      if (r < 0 || r >= height_ || c < 0 || c >= width_) {
        continue;
      }
      double new_dis = current_dis + std::sqrt(bias.first * bias.first + bias.second * bias.second);
      if (map2d_[r * width_ + c] < new_dis) {
        continue;
      }
      que.erase(std::make_pair(r, c));
      map2d_[r * width_ + c] = new_dis;
      que.insert(std::make_pair(r, c));
    }
  }
  std::cout << "PreCalcDistMap: End" << std::endl;
}

double Map2D::MinDist2Edge(Eigen::Vector3d pt) const {
  Eigen::Matrix<double, 3, 4> P;
  P.block(0, 0, 3, 3) = R_;
  P.block(0, 3, 3, 1) = T_;
  P = K_ * P;
  Eigen::Matrix<double, 4, 1> X;
  X.block(0, 0, 3, 1) = pt;
  X(3, 0) = 1.0;
  auto pix = P * X;
  double i_d = pix(1, 0);
  double j_d = pix(0, 0);
  if (i_d < 1e-8 || i_d > height_ - (1.0 + 1e-8) || j_d < 1e-8 || j_d > width_ - (1.0 + 1e-8)) {
    return 1e7;
  } 
  int i = static_cast<int>(i_d);
  int j = static_cast<int>(j_d);
  double res_i = i_d - i;
  double res_j = j_d - j;
  double dis = (map2d_[i * width_ + j] * (2.0 - res_i - res_j) +
                map2d_[i * width_ + j + 1] * (1.0 - res_i + res_j) +
                map2d_[(i + 1) * width_ + j] * (res_i + 1.0 - res_j) +
                map2d_[(i + 1) * width_ + j + 1] * (res_i + res_j)) / 4.0;
  return dis;
}