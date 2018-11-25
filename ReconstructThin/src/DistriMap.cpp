#include "DistriMap.h"
#include <fstream>
#include <cstring>
#include <iostream>

DistriMap::DistriMap(const cv::Mat &img, bool calc_distri_map, std::string map_path) {
  std::cout << "DistriMap Reconstruction: Begin" << std::endl;
  height_ = img.rows;
  width_ = img.cols;
  distri_map_ = new double[height_ * width_];
  point_map_ = new int[height_ * width_];
  std::memset(point_map_, 0, height_ * width_ * sizeof(int));
  num_point_ = 0;
  if (calc_distri_map) {
    CalcDistriMap(img, map_path);
  }
  else {
    LoadDistriMap(map_path);
  }
  CalcSumDistri();
}

DistriMap::~DistriMap() {
  delete(distri_map_);
  delete(point_map_);
}

void DistriMap::AddPoint(Eigen::Vector3d pt) {
  // We should guarantee no point is outside the image.
  if (num_point_ > 0) {
    for (int i = 0; i < width_ * height_; i++) {
      current_cost_ -=
        std::abs(
          static_cast<double>(point_map_[i]) / static_cast<double>(num_point_) -
          distri_map_[i] / sum_distri_
        );
    }
  }
  num_point_++;
  auto mapped = Map3to2(pt);
  int a = static_cast<int>(std::round(mapped(0)));
  int b = static_cast<int>(std::round(mapped(1)));
  if (a < 0 || a >= height_ || b < 0 || b >= width_) {
    current_cost_ += static_cast<double>(
      std::max(0, std::max(0 - a, a - height_)) + std::max(0, std::max(0 - b, b - width_)));
  } else {
    point_map_[a * width_ + b]++;
  }
  for (int i = 0; i < width_ * height_; i++) {
    current_cost_ +=
      std::abs(
        static_cast<double>(point_map_[i]) / static_cast<double>(num_point_) -
        distri_map_[i] / sum_distri_
      );
  }
}

/*void DistriMap::DeletePoint(Eigen::Vector3d pt) {
  current_cost_ = 0;
  num_point_--;
  auto mapped = Map3to2(pt);
  int a = static_cast<int>(std::round(mapped(0)));
  int b = static_cast<int>(std::round(mapped(1)));
  if (a >= 0 && a < height_ && b >= 0 && b < width_) {
    point_map_[a * width_ + b]--;
  } else {
    current_cost_ -= static_cast<double>(
      std::max(0.0, std::max(0 - a, a - height_)) + std::max(0.0, std::max(0 - b, b - width_)));
  }
  for (int i = 0; i < width_ * height_; i++) {
    current_cost_ +=
      std::abs(
        static_cast<double>(point_map_[i]) / static_cast<double>(num_point_) -
        distri_map_[i] / sum_distri_
      );
  }
}*/

void DistriMap::ChangePoint(Eigen::Vector3d past_pt, Eigen::Vector3d pt) {
  auto mapped1 = Map3to2(past_pt);
  int a1 = static_cast<int>(std::round(mapped1(0)));
  int b1 = static_cast<int>(std::round(mapped1(1)));  
  auto mapped2 = Map3to2(pt);
  int a2 = static_cast<int>(std::round(mapped2(0)));
  int b2 = static_cast<int>(std::round(mapped2(1)));
  if (a1 == b1 && a2 == b2) {
    return;
  }
  if (a1 >= 0 && a1 < height_ && b1 >= 0 && b1 < width_) {
    current_cost_ -= std::abs(
      static_cast<double>(point_map_[a1 * width_ + b1]) / static_cast<double>(num_point_) -
      distri_map_[a1 * width_ + b1] / sum_distri_
    );
    point_map_[a1 * width_ + b1]--;
    current_cost_ += std::abs(
      static_cast<double>(point_map_[a1 * width_ + b1]) / static_cast<double>(num_point_) -
      distri_map_[a1 * width_ + b1] / sum_distri_
    );
  }
  else {
    current_cost_ -= static_cast<double>(
      std::max(0, std::max(0 - a1, a1 - height_)) + std::max(0, std::max(0 - b1, b1 - width_)));
  }

  if (a2 >= 0 && a2 < height_ && b2 >= 0 && b2 < width_) {
    current_cost_ -= std::abs(
      static_cast<double>(point_map_[a2 * width_ + b2]) / static_cast<double>(num_point_) -
      distri_map_[a2 * width_ + b2] / sum_distri_
    );
    point_map_[a2 * width_ + b2]++;
    current_cost_ += std::abs(
      static_cast<double>(point_map_[a2 * width_ + b2]) / static_cast<double>(num_point_) -
      distri_map_[a2 * width_ + b2] / sum_distri_
    );
  } else {
    current_cost_ += static_cast<double>(
      std::max(0, std::max(0 - a2, a2 - height_)) + std::max(0, std::max(0 - b2, b2 - width_)));
  }
  return;
}

double DistriMap::CalcCost() {
  return current_cost_;
}

void DistriMap::CalcDistriMap(const cv::Mat &img, std::string map_path) {
  std::cout << "CalcDistriMap: Begin" << std::endl;
  auto dist_map = new double[height_ * width_];
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
      if (!IsInside(img, i, j)) {
        continue;
      }
      bool near_outside = false;
      for (int a_ = -1; a_ <= 1; a_++) {
        int i_ = i + a_;
        if (i_ < 0 || i_ >= height_)
          continue;
        for (int b_ = -1; b_ <= 1; b_++) {
          int j_ = j + b_;
          if (j_ < 0 || j_ >= width_)
            continue;
          if (!IsInside(img, i_, j_)) {
            near_outside = true;
          }
        }
      }
      if (near_outside) {
        dist_map[i * width_ + j] = 0.5;
        que.insert(std::make_pair(i, j));
      }
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

  sum_distri_ = 0;
  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < width_; j++) {
      if (IsInside(img, i, j)) {
        distri_map_[i * width_ + j] = 1.0 / dist_map[i * width_ + j];
      } else {
        distri_map_[i * width_ + j] = 1e-4 / dist_map[i * width_ + j]; 
      }
    }
  }
  // Save bin
  std::ofstream tmp_file;
  tmp_file.open(map_path.c_str(), std::ios::binary | std::ios::ate);
  tmp_file.write((const char *) distri_map_, width_ * height_ * sizeof(double));
  tmp_file.close();

  delete(dist_map);
  std::cout << "CalcDistriMap: End" << std::endl;
}

void DistriMap::LoadDistriMap(std::string map_path) {
  std::ifstream map_stream(map_path.c_str(), std::ios::binary | std::ios::ate);
  map_stream.seekg(0, std::ios::beg);
  map_stream.read((char *)(distri_map_), width_ * height_ * sizeof(double));
  map_stream.close();
}

void DistriMap::CalcSumDistri() {
  sum_distri_ = 0.0;
  for (int i = 0; i < width_ * height_; i++) {
    sum_distri_ += distri_map_[i];
  }
}

void DistriMap::SetKRT(Eigen::Matrix3d K, Eigen::Matrix3d R, Eigen::Vector3d T) {
  K_ = K;
  R_ = R;
  T_ = T;
}

Eigen::Vector2d DistriMap::Map3to2(Eigen::Vector3d pt) {
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

bool DistriMap::IsInside(const cv::Mat &img, int i, int j) {
  uint8_t *img_ptr = (uint8_t*) img.data;
  return (static_cast<int>(img_ptr[(i * width_ + j) * 3]) != 0 ||
          static_cast<int>(img_ptr[(i * width_ + j) * 3 + 1]) != 0 ||
          static_cast<int>(img_ptr[(i * width_ + j) * 3 + 2]) != 0);
}