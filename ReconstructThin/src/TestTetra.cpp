#include "Tetra.h"
#include "Utils.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>

double ran() {
  return static_cast<double>(std::abs(rand()) % 100000) / 100000.0; 
}

int main() {
  std::vector<Eigen::Vector3d> points;

  //srand(444);
  for (int num = 1000; num; num--) 
    points.emplace_back(ran(), ran(), ran());
  auto tetras = new std::vector<Tetra>();
  Tetrahedralization(points, tetras);
  std::cout << "tetra num ! " << tetras->size() << std::endl;
  std::vector<Trian> trians;
  for (const auto tetra : *tetras) {
    for (int i = 0; i < 4; i++)
      for (int j = i + 1; j < 4; j++)
        for (int k = j + 1; k < 4; k++) {
          trians.emplace_back(tetra.points_[i], tetra.points_[j], tetra.points_[k]);
        }
  }
  Utils::SaveTriansAsPly(std::string("./mesh.ply"), trians);
  return 0;
}