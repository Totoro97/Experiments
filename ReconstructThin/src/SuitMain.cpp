#include "Headers.h"
#include "Algorithms.h"
#include "DistriMap.h"
#include "Utils.h"
#include "Tetra.h"
#include <vector>
#include <string>

int main() {
  std::vector<Eigen::Matrix3d> Ks, Rs;
  std::vector<Eigen::Vector3d> Ts;
  int num_cam = 5;
  Utils::ReadKRTFromFile(std::string("/home/totoro/tmp/Cameras.txt"), num_cam, &Ks, &Rs, &Ts);
  std::vector<DistriMap *> distri_maps;
  for (int i = 0; i < num_cam; i++) {
    auto img =
      cv::imread(std::string("/home/totoro/tmp/") + std::to_string(i) + std::string(".png"));
    // cv::imshow("img", img);
    // cv::waitKey(100);
    auto tmp_ptr = new DistriMap(img, true, std::to_string(i) + std::string(".bin"));
    distri_maps.push_back(tmp_ptr);
    // std::fflush(stdout);
    distri_maps[i]->SetKRT(Ks[i], Rs[i], Ts[i]);
  }
  std::cout << "SuitWithPointCloud: Begin" << std::endl;
  std::vector<Eigen::Vector3d> points;
  Algo::SuitWithPointCloud(distri_maps, points, 10000, 1000, 2.0);
  Utils::SavePoints("points.json", points);
  for (int i = 0; i < num_cam; i++) {
  auto distri_ptr = distri_maps[i];
    Utils::SaveGrayScaleImageFromPtr(
      distri_ptr->distri_map_,
      distri_ptr->width_,
      distri_ptr->height_,
      std::to_string(i) + std::string("just.png")
    );
  }

  std::cout << "------------------Tetra: Begin------------------" << std::endl;
  std::cout << "Point size: " << points.size() << std::endl;
  auto tetras = new std::vector<Tetra>();
  std::vector<Trian> trians;
  Tetrahedralization(points, tetras);
  for (const auto tetra : *tetras) {
    double s = 0;
    Eigen::Vector3d pt(0.0, 0.0, 0.0);
    for (int i = 0; i < 4; i++) {
      pt += tetra.points_[i];
    }
    pt *= 0.25;
    
    /*
    double bias = 0.0;
    for (int i = 0; i < 4; i++) {
      bias += (pt - tetra.points_[i]).norm();
    }
    std::cout << "bias = " << bias << std::endl;
    if (bias > 1) {
      continue;
    }*/
    bool is_ok = true;
    for (auto &distri_map : distri_maps) {
      auto tmp = distri_map->Map3to2(pt);
      int a = static_cast<int>(std::round(tmp(0)));
      int b = static_cast<int>(std::round(tmp(1)));
      if (a < 0 || a >= distri_map->height_ || b < 0 || b >= distri_map->width_) {
        is_ok = false;
        break;
      }
      if (distri_map->distri_map_[a * distri_map->width_ + b] > 1 * 1e4) {
        is_ok = false;
        break;
      }
    }
    if (!is_ok) {
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
  return 0;
}