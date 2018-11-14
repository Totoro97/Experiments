#include "Map2D.h"

// std
#include <algorithm>
#include <cmath>
#include <pair>
#include <queue>
#include <set>

Map2D::~Map2D() {
  delete(map2d_);
}

Map2D::Map2D(const cv::mat& img) {
  height_ = img.rows;
  width_ = img.cols;
  map2d_ = new double[height_ * width_];
  std::fill_n(map2d_, height_ * width_, 1e9);
  ed::Ed ed;
  auto edges = ed.detectEdges(img);

  auto dis_cmp = [map2d_, width_](std::pair<int, int> a, std::pair<int, int> b) {
    double dist_a = map2d_[a.first * width_ + a.second];
    double dist_b = map2d_[b.first * width_ + b.second];
    if (dist_a < dist_b - 1e-8)
      return true;
    if (dist_a > dist_b + 1e-8)
      return false;
    return (a.first * width_ + a.second < b.first * width_ + b.second);
  };

  std::set<double, dis_cmp> que;
  for (const auto &edge : edges) {
    for (const auto &pix : edge) {
      map2d_[pix.y * width_ + pix.x] = 0;
      que.insert(std::make_pair(pix.y, pix.x));
    }
  }

  // dist map
  std::vector<std::pair<int, int> > biases;
  const int lim = 50;
  for (int i = -lim; i < lim; i++)
    for (int j = -lim; j < lim; j++) {
      if (__gcd(std::abs(i), std::abs(j)) == 1) {
        biases.push_back(std::make_pair(i, j));
      }
    }
  
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
}

double Map2d::MinDist2Edge(Eigen::Vector3d pt) const {
  Eigen::Matrix<3, 4, double> P;
  P.block(0, 0, 3, 3) = R_;
  P.block(0, 3, 3, 1) = T_;
  P = K_ * P;
  Eigen::Matrix<4, 1, double> X;
  X.block(0, 0, 3, 1) = pt;
  X(3, 0) = 1.0;
  auto pix = P * X;
  int i = static_cast<int>(pix(0, 1));
  int j = static_cast<int>(pix(0, 0));
  if (i < 0 || i >= height_ || j < 0 || j >= width_) {
    return 1e7;
  }
  return map2d_[i * width_ + j];
}