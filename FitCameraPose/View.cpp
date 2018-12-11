#include "View.h"
#include "Utils.h"
#include <algorithm>
#include <fstream>
#include <cstring>
#include <iostream>

View::View(std::string image_path, bool calc_dist_map, std::string map_path) {
  img_ = cv::imread(image_path);
  std::cout << "View Reconstruction: Begin" << std::endl;
  height_ = img_.rows;
  width_ = img_.cols;
  dist_map_ = new double[height_ * width_];
  if (calc_dist_map) {
    CalcDistMap(img_, map_path);
  }
  else {
    LoadDistMap(map_path);
  }
}

View::~View() {
  delete(dist_map_);
}

void View::SetKRT(Eigen::Matrix3d K, Eigen::Matrix3d R, Eigen::Vector3d T) {
  K_ = K;
  R_ = R;
  T_ = T;
  Eigen::Matrix4d world_to_view;
  world_to_view.block(0, 0, 3, 3) = R_;
  world_to_view.block(0, 3, 3, 1) = T_;
  world_to_view.block(3, 0, 1, 4) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0).transpose();
  view_to_world_ = world_to_view.inverse();
}

void View::CalcDistMap(const cv::Mat &img, std::string map_path) {
  std::cout << "CalcMap: Begin" << std::endl;
  auto dist_map = dist_map_;
  std::fill_n(dist_map, width_ * height_, 1e9);
  auto dis_cmp = [dist_map, this](std::pair<int, int> a, std::pair<int, int> b) {
    double dist_a = dist_map[a.first * this->width_ + a.second];
    double dist_b = dist_map[b.first * this->width_ + b.second];
    if (dist_a < dist_b - 1e-8)
      return true;
    if (dist_a > dist_b + 1e-8)
      return false;
    return (a.first * this->width_ + a.second < b.first * this->width_ + b.second);
  };

  std::set<std::pair<int, int>, decltype(dis_cmp)> que(dis_cmp);
  que.clear();
  // Calc
  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < width_; j++) {
      if (!IsInside(i, j)) {
        continue;
      }
      dist_map[i * width_ + j] = 0;
      que.insert(std::make_pair(i, j));
    }
  }

  std::vector<std::pair<int, int> > biases;
  const int lim = 5;
  std::function<int(int, int)> gcd = [&gcd](int a, int b) {
    return b ? gcd(b, a % b) : a;
  };
  for (int i = -lim; i <= lim; i++)
    for (int j = -lim; j <= lim; j++) {
      if (gcd(std::abs(i), std::abs(j)) == 1) {
        biases.push_back(std::make_pair(i, j));
      }
    }
  std::cout << "biased size = " << biases.size() << std::endl;
  while (!que.empty()) {
    auto pix = *que.begin();
    que.erase(que.begin());
    if (dist_map[pix.first * width_ + pix.second] > 50) {
      break;
    }
    double current_dis = dist_map[pix.first * width_ + pix.second];
    for (const auto &bias : biases) {
      int r = pix.first + bias.first;
      int c = pix.second + bias.second;
      if (r < 0 || r >= height_ || c < 0 || c >= width_) {
        continue;
      }
      double new_dis = current_dis + std::sqrt(bias.first * bias.first + bias.second * bias.second);
      if (dist_map[r * width_ + c] < new_dis) {
        continue;
      }
      que.erase(std::make_pair(r, c));
      dist_map[r * width_ + c] = new_dis;
      que.insert(std::make_pair(r, c));
    }
  }

  // Save bin
  std::ofstream tmp_file;
  tmp_file.open(map_path.c_str(), std::ios::binary | std::ios::ate);
  tmp_file.write((const char *) dist_map_, width_ * height_ * sizeof(double));
  tmp_file.close();

  std::cout << "CalcDistMap: End" << std::endl;
}

void View::LoadDistMap(std::string map_path) {
  std::ifstream map_stream(map_path.c_str(), std::ios::binary | std::ios::ate);
  map_stream.seekg(0, std::ios::beg);
  map_stream.read((char *)(dist_map_), width_ * height_ * sizeof(double));
  map_stream.close();
}

bool View::IsInside(int i, int j) {
  uint8_t *img_ptr = (uint8_t*) img_.data;
  return (static_cast<int>(img_ptr[(i * width_ + j) * 3]) != 0 ||
          static_cast<int>(img_ptr[(i * width_ + j) * 3 + 1]) != 0 ||
          static_cast<int>(img_ptr[(i * width_ + j) * 3 + 2]) != 0);
}

double View::Distance(Eigen::Vector3d pt) {
  auto tmp = Map3to2(pt);
  double i_d = tmp(0);
  double j_d = tmp(1);
  double distance = 0.0;
  if (i_d < 1e-4 || i_d > height_ - 1.0 - 1e-4) {
    distance += std::max(-i_d, i_d - (height_ - 1));
    i_d = std::max(1e-4, std::min(i_d, height_ - 1.0 - 1e-4));
  }
  if (j_d < 1e-4 || j_d > width_ - 1.0 - 1e-4) {
    distance += std::max(-j_d, j_d - (width_ - 1));
    j_d = std::max(1e-4, std::min(j_d, width_ - 1.0 - 1e-4));
  }
  int i = static_cast<int>(i_d);
  int j = static_cast<int>(j_d);
  double i_res = i_d - static_cast<double>(i);
  double j_res = j_d - static_cast<double>(j);
  double sum = 0.0, sum_weight = 0.0;
  for (int bias_i = 0; bias_i < 2; bias_i++)
    for (int bias_j = 0; bias_j < 2; bias_j++) {
      double weight = 1.0 / (1e-4 + std::abs(i_res - bias_i) + std::abs(j_res - bias_j));
      sum_weight += weight;
      sum += dist_map_[(i + bias_i) * width_ + j + bias_j] * weight;
    }
  return distance + sum / sum_weight;
}


Eigen::Vector2d View::Map3to2(Eigen::Vector3d pt) {
  Eigen::Matrix<double, 3, 4> P;
  P.block(0, 0, 3, 3) = R_;
  P.block(0, 3, 3, 1) = T_;
  P = K_ * P;
  Eigen::Matrix<double, 4, 1> X;
  X.block(0, 0, 3, 1) = pt;
  X(3, 0) = 1.0;
  auto pix = P * X;
  double i_d = pix(1, 0) / pix(2, 0);
  double j_d = pix(0, 0) / pix(2, 0);
  return Eigen::Vector2d(i_d, j_d);
}

Eigen::Vector3d View::Map2to3(double a, double b, double depth) {
  double x = (b - K_(0, 2)) / K_(0, 0) * depth;
  double y = (a - K_(1, 2)) / K_(1, 1) * depth;
  double z = depth;
  Eigen::Matrix<double, 4, 1> coord(x, y, z, 1.0);
  coord = view_to_world_ * coord;
  return coord.block(0, 0, 3, 1);
}

void View::SamplePoints(std::vector<Eigen::Vector3d> &points, double weight) {
  bool *done = new bool[width_ * height_];
  std::memset(done, 0, width_ * height_);
  for (const auto &pt : points) {
    auto tmp = Map3to2(pt);
    int i = static_cast<int>(std::round(tmp(0)));
    int j = static_cast<int>(std::round(tmp(1)));
    if (i >= 0 && i < height_ && j >= 0 && j < width_) {
      done[i * width_ + j] = true;
    }
  }
  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < width_; j++) {
      if (!done[i * width_ + j] && IsInside(i, j) && Utils::Random() < weight) {
        points.push_back(Map2to3(i, j, 5.0));
      }
    }
  }
  delete[](done);
}