#include "Utils.h"
// std
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <map>

void Utils::ReadKRTFromFile(std::string file_path, int cam_num,
                     std::vector<Eigen::Matrix3d> *Ks,
                     std::vector<Eigen::Matrix3d> *Rs,
                     std::vector<Eigen::Vector3d> *Ts) {
  Ks->clear();
  Rs->clear();
  Ts->clear();
  FILE *f = fopen(file_path.c_str(), "r");
  for (int i = 0; i < cam_num; i++) {
    // K, R, t
    Eigen::Matrix3d K, R;
    Eigen::Vector3d T;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double val;
        fscanf(f, "%lf", &val);
        K(i, j) = val;
      }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double val;
        fscanf(f, "%lf", &val);
        R(i, j) = val;
      }
    for (int i = 0; i < 3; i++) {
      double val;
      fscanf(f, "%lf", &val);
      T(i) = val;
    }
    Ks->push_back(K);
    Rs->push_back(R);
    Ts->push_back(T);
  }
  fclose(f); 
}


void Utils::SaveTriansAsPly(std::string save_path, const std::vector<Trian> &trians) {
  auto cmp = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    for (int i = 0; i < 3; i++) {
      if (std::abs(a(i) - b(i)) > 1e-6) {
        return (a(i) < b(i));
      }
    }
    return false;
  };
  std::map<Eigen::Vector3d, int, decltype(cmp)> mp(cmp);
  for (const auto &trian : trians) {
    for (int i = 0; i < 3; i++) {
      mp[trian.points_[i]] = 0;
    }
  }
  int cnt = 0;
  for (auto iter = mp.begin(); iter != mp.end(); iter++) {
    iter->second = cnt++;
  }
  std::ofstream my_file;
  my_file.open(save_path.c_str());
  my_file << "ply\nformat ascii 1.0\n";
  my_file << "element vertex " << mp.size() << "\n";
  my_file << "property float32 x\nproperty float32 y\nproperty float32 z\n";
  my_file << "element face " << trians.size() << "\n";
  my_file << "property list uint8 int32 vertex_indices\n";
  my_file << "end_header\n";
  for (auto iter = mp.begin(); iter != mp.end(); iter++) {
    my_file << (iter->first)(0) << " " << (iter->first)(1) << " " << (iter->first)(2) << "\n";
  }
  for (const auto &trian : trians) {
    my_file << "3 ";
    for (int i = 0; i < 3; i++) {
      my_file << mp[trian.points_[i]];
      if (i < 2) {
        my_file << " ";
      } else {
        my_file << "\n";
      }
    }
  }
  my_file.close();
}


void Utils::Srand() {
  srand(123);
}

double Utils::Random() {
  return static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
}

double Utils::RandomLR(double l, double r) {
  return l + Random() * (r - l);
}


void Utils::SavePoints(std::string save_path, const std::vector<Eigen::Vector3d> &points) {
  std::ofstream my_file;
  my_file.open(save_path.c_str());
  my_file << "{\n\"points\": [";
  for (auto pt = points.begin(); pt != points.end(); pt++) {
    my_file << "[" << (*pt)(0) << "," << (*pt)(1) << "," << (*pt)(2) << "]";
    if (std::next(pt) == points.end()) {
      my_file << "]\n";
    } else {
      my_file << ",\n";
    }
  }
  my_file << "}\n";
}

void Utils::SaveGrayScaleImageFromPtr(double *pt,
                                      int width,
                                      int height,
                                      std::string file_path) {
  double val_min = 1e9;
  double val_max = -1e9;
  for (int i = 0; i < width * height; i++) {
    val_min = std::min(val_min, pt[i]);
    val_max = std::max(val_max, pt[i]);
  }
  uint8_t *tmp_ptr = new uint8_t[width * height];
  for (int i = 0; i < width * height; i++) {
    tmp_ptr[i] = static_cast<uint8_t>((pt[i] - val_min) / (val_max - val_min) * 255.0);
  }
  cv::Mat tmp_mat(cv::Size(width, height), CV_8UC1, (void *) tmp_ptr);
  cv::imwrite(file_path.c_str(), tmp_mat);
  delete(tmp_ptr);
}