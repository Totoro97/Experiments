#pragma once

#include <Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Octree.h"

class Calibrator {
public:
  Calibrator(const cv::Mat &img_gray, Octree* octree, int frame_id = 0, double ang = -1.0);

  ~Calibrator() {
  }

  void AddToOctree();
  void Calibrate();
  void CopyCamParasFrom(Calibrator *another_calibrator);
  double CalcCurrentF();

  int frame_id_;
  int height_, width_;
  cv::Mat img_gray_;
  Octree* octree_;
  std::vector<Eigen::Vector2d> visible_pixs_;
  Eigen::Vector3d pos_, x_axis_, y_axis_, z_axis_;
  double scale_ratio_ = 1050.0;
};
