//
// Created by aska on 19-2-1.
//

#include "Utils.h"
#include <cstdlib>

void Utils::SavePointsAsPly(std::string save_path, const std::vector<Eigen::Vector3d> &points) {
  std::ofstream my_file;
  my_file.open(save_path.c_str());
  my_file << "ply\nformat ascii 1.0\n";
  my_file << "element vertex " << points.size() << "\n";
  my_file << "property float32 x\nproperty float32 y\nproperty float32 z\n";
  my_file << "end_header\n";
  for (const auto &pt : points) {
    my_file << pt(0) << " " << pt(1) << " " << pt(2) << "\n";
  }
  my_file.close();
}

double Utils::Random() {
  return (double) std::rand() / (double) RAND_MAX;
}

double Utils::RandomLR(double l, double r) {
  return l + (r - l) * Random();
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

Trian::Trian(Eigen::Vector3d pt0, Eigen::Vector3d pt1, Eigen::Vector3d pt2, bool want_to_sort) {
  points_[0] = pt0;
  points_[1] = pt1;
  points_[2] = pt2;
  if (!want_to_sort) {
    return;
  }
  auto cmp = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    for (int i = 0; i < 3; i++) {
      if (std::abs(a(i) - b(i)) > 1e-5) {
        return (a(i) < b(i));
      }
      return false;
    }
  };
  std::sort(points_, points_ + 3, cmp);
}