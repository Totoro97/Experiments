//
// Created by aska on 19-2-1.
//

#pragma once
#include <Eigen>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <fstream>

class Trian {
public:
  Eigen::Vector3d points_[3];

  Trian(Eigen::Vector3d pt0, Eigen::Vector3d pt1, Eigen::Vector3d pt2, bool want_to_sort = false);
};

namespace Utils {
  void SavePointsAsPly(std::string save_path, const std::vector<Eigen::Vector3d> &points);
  double RandomLR(double l, double r);
  double Random();
  void SaveTriansAsPly(std::string save_path, const std::vector<Trian> &trians);
}

