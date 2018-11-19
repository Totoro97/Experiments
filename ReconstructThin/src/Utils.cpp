#include "Utils.h"
// Eigen
#include <eigen3/Eigen/Eigen>
// std
#include <algorithm>
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
      if (std::abs(a(i) - b(i)) > 1e-3) {
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