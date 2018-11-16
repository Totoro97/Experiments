#pragma once
#include "LineGenerator.h"
#include "Map2D.h"
#include "Curve3d.h"

// Eigen
#include <eigen3/Eigen/Eigen>

// std
#include <vector>

class Bend {
public:
  Bend(LineGenerator *line_generator, std::vector<Map2D *> *map2ds): line_generator_(line_generator),
      map2ds_(map2ds) {}
  void GoBendNow();

  LineGenerator *line_generator_;
  std::vector<Map2D *> *map2ds_;
  
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* lines_;
  std::vector<Curve3d> curves_;
};