#pragma once
#include <Eigen>
#include <string>
#include <vector>

#include "Calibrator.h"
#include "Octree.h"

class GlobalCalibrator {
public:
  GlobalCalibrator(std::string dir_name, int num_frame);
  ~GlobalCalibrator();
  void Run();

  std::vector<Calibrator *> calibrators_;
  Octree* octree_;
  int num_frame_;
};