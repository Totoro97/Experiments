#pragma once
#include "Headers.h"
#include <string>

class DistMap {
public:
  DistMap(const cv::Mat &img, bool calc_dist_map = false, std::string map_path = ".");
  ~DistMap();
  void CalcDistMap(const cv::Mat &img, std::string map_path);
  void LoadDistMap(std::string map_path);
  double Distance(double a, double b);
  bool IsInside(const cv::Mat &img, int i, int j);
  
  int height_;
  int width_;
  double *dist_map_;
};