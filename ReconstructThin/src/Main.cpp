// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// Eigen
#include <eigen3/Eigen/Eigen>
// std
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <list>
// self
#include "LineGenerator.h"
#include "Map2D.h"
#include "Bend.h"

int main() {
  auto line_generator = new LineGenerator("../data", 64);
  line_generator -> GenerateLine();

  std::vector<Map2D *> map2ds;
  for (int i = 0; i < 64; i++) {
    auto img_path = std::string("../data/") + std::to_string(i) + std::string(".png");
    cv::Mat img = cv::imread(img_path);
    auto tmp_ptr = new Map2D(img);
    map2ds.push_back(tmp_ptr);
  }

  auto bend = new Bend(line_generator, &map2ds);
  bend -> GoBendNow();
  return 0;
}