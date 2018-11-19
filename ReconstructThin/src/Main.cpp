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
#include "Utils.h"

void OutPutCurve(Bend *bend) {
  std::string save_path = std::string("curves.json");
  FILE *f = fopen(save_path.c_str(), "w");
  fprintf(f, "{\n \"curves\": [");
  for (auto curve = bend->curves_.begin(); curve != bend->curves_.end(); curve++) {
    fprintf(f, "[");
    for (auto pt = curve->points_.begin(); pt != curve->points_.end(); pt++) {
      fprintf(f, "[");
      fprintf(f, "%.4lf, %.4lf, %.4lf", (*pt)(0), (*pt)(1), (*pt)(2));
      fprintf(f, "]");
      if (pt != std::prev(curve->points_.end())) {
        fprintf(f, ", ");
      }
    }
    if (curve == std::prev(bend->curves_.end())) {
      fprintf(f, "]]\n");
    }
    else {
      fprintf(f, "],\n");
    }
  }
  fprintf(f, "}\n");
  fclose(f);
}

int main() {
  std::cout << "Gen3DLine: Begin" << std::endl;
  auto line_generator = new LineGenerator("../data", 64);
  line_generator -> GenerateLine();

  std::cout << "-----------------Gen3DLine: End-----------------" << std::endl;
  std::cout << "-----------------Gen2DMap: Begin----------------" << std::endl;
  std::vector<Map2D *> map2ds;
  std::vector<Eigen::Matrix3d> Ks, Rs;
  std::vector<Eigen::Vector3d> Ts;
  Utils::ReadKRTFromFile(std::string("../data/Cameras.txt"), 64, &Ks, &Rs, &Ts);
  for (int i = 0; i < 64; i++) {
    auto img_path = std::string("../data/") + std::to_string(i) + std::string(".png");
    cv::Mat img = cv::imread(img_path);
    auto tmp_ptr = new Map2D(img, false, "./map2d_" + std::to_string(i) + ".bin");
    tmp_ptr->SetKRT(Ks[i], Rs[i], Ts[i]);
    //std::cout << tmp_ptr->K_ << std::endl;
    //std::cout << tmp_ptr->R_ << std::endl;
    //std::cout << tmp_ptr->T_ << std::endl;
    //exit(0);
    map2ds.push_back(tmp_ptr);
  }

  std::cout << "-----------------Gen2DMap: End------------------" << std::endl;
  std::cout << "-----------------Bend: Begin--------------------" << std::endl;
  auto bend = new Bend(line_generator, &map2ds);
  bend->GoBendNow();
  OutPutCurve(bend);
  std::cout << "------------------Bend: End---------------------" << std::endl;
  std::cout << "------------------Tetra: Begin------------------" << std::endl;
  std::vector<Eigen::Vector3d> points;
  for (const auto &curve : bend->curves_) {
    for (const auto &pt : curve.points_) {
      points.push_back(pt);
    }
  }
  std::cout << "Point size: " << points.size() << std::endl;
  auto tetras = new std::vector<Tetra>();
  Tetrahedralization(points, tetras);
  std::cout << "tetras size ! " << tetras->size() << std::endl;
  std::vector<Trian> trians;
  for (const auto tetra : *tetras) {
    double s = 0;
    auto pt = tetra.sphere_center_;
    for (auto &map2d : map2ds) {
      s += map2d->MinDist2Edge(pt);
    }
    if (s > 1500) {
      continue;
    }
    for (int i = 0; i < 4; i++)
      for (int j = i + 1; j < 4; j++)
        for (int k = j + 1; k < 4; k++) {
          auto vec =
            (tetra.points_[j] - tetra.points_[i]).cross(tetra.points_[k] - tetra.points_[i]);
          int p = 0;
          while (p == i || p == j || p == k)
            p++;
          if (vec.dot(tetra.points_[p] - tetra.points_[i]) > 0.0) {
            trians.emplace_back(tetra.points_[i], tetra.points_[k], tetra.points_[j], false);
          }
          else {
            trians.emplace_back(tetra.points_[i], tetra.points_[j], tetra.points_[k], false);
          }
        }
  }
  Utils::SaveTriansAsPly(std::string("./mesh.ply"), trians);
  std::cout << "------------------Tetra: End--------------------" << std::endl; 
  return 0;
}