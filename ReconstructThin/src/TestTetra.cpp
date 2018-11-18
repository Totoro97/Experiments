#include "Tetra.h"

#include <vector>

int main() {
  std::vector<Eigen::Vector3d> points;

  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      for (int k = 0; k < 2; j++) 
        points.emplace_back(static_cast<double>(i), static_cast<double>(j), static_cast<double>(k));
  
  auto tetras = new std::vector<Tetra>();
  Tetrahedralization(points, tetras);
  return 0;
}