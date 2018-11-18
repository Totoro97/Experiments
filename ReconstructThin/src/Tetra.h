#pragma once
// Eigen
#include <eigen3/Eigen/Eigen>

class Tetra {
public:
  Eigen::Vector3d points_[4];
  Eigen::Vector3d sphere_center_;
  double sphere_radius_ = 0;

  Tetra();
  Tetra(Eigen::Vector3d pt0, Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3, 
        bool calc_sphere = true);

  Tetra(Eigen::Vector3d* points, bool calc_sphere = true);

  void CalcSphere();
  bool InSphere(Eigen::Vector3d pt);
};

class Trian {
public:
  Eigen::Vector3d points_[3];

  Trian(Eigen::Vector3d pt0, Eigen::Vector3d pt1, Eigen::Vector3d pt2);
};

void Tetrahedralization(const std::vector<Eigen::Vector3d>& points, std::vector<Tetra> *tetras);