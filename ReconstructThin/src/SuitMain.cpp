#include "Headers.h"
#include "Algorithms.h"
#include "DistriMap.h"
#include "Utils.h"
#include <vector>
#include <string>

int main() {
  std::vector<Eigen::Matrix3d> Ks, Rs;
  std::vector<Eigen::Vector3d> Ts;
  Utils::ReadKRTFromFile(std::string("/home/totoro/tmp/Cameras.txt"), 64, &Ks, &Rs, &Ts);
  std::vector<DistriMap *> distri_maps;
  for (int i = 0; i < 64; i++) {
    auto img =
      cv::imread(std::string("/home/totoro/tmp/") + std::to_string(i) + std::string(".png"));
    // cv::imshow("img", img);
    // cv::waitKey(100);
    auto tmp_ptr = new DistriMap(img, false, std::to_string(i) + std::string(".bin"));
    distri_maps.push_back(tmp_ptr);
    // std::fflush(stdout);
    distri_maps[i]->SetKRT(Ks[i], Rs[i], Ts[i]);
  }
  std::cout << "SuitWithPointCloud: Begin" << std::endl;
  std::vector<Eigen::Vector3d> points;
  Algo::SuitWithPointCloud(distri_maps, points, 2000, 1000, 2.0);
  Utils::SavePoints("points.json", points);
  for (int i = 0; i < 64; i++) {
  auto distri_ptr = distri_maps[i];
    Utils::SaveGrayScaleImageFromPtr(
      distri_ptr->distri_map_,
      distri_ptr->width_,
      distri_ptr->height_,
      std::to_string(i) + std::string("just.png")
    );
  }
  return 0;
}