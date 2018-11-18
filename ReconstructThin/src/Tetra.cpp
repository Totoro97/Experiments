#include "Tedra.h"

// std
#include <algorithm>
#include <cassert>
#include <set>

Tetra::Tetra(Eigen::Vector3d pt0, Eigen::Vector3d pt1, Eigen::Vector3d pt2, Eigen::Vector3d pt3, 
             bool calc_sphere) {
  points_[0] = pt0;
  points_[1] = pt1;
  points_[2] = pt2;
  points_[3] = pt3;
  if (calc_sphere) {
    this->CalcSphere();
  }
}

Tetra::Tetra(Eigen::Vector3d* points, bool calc_sphere) {
  for (int i = 0; i < 4; i++) {
    points_[i] = points[i];
  }
  if (calc_sphere) {
    this->CalcSphere();
  }
}

void Tetra::CalcSphere() {
  Matrix4f matrix_a;
  matrix_a << points_[0](0), points_[0](1), points_[0][2], 1.0,
              points_[1](0), points_[1](1), points_[1][2], 1.0,
              points_[2](0), points_[2](1), points_[2][2], 1.0,
              points_[3](0), points_[3](1), points_[3][2], 1.0,
  double a = matrix_a.determinant();
  
  auto square_sum = [](double a, double b, double c){
    return a * a + b * b + c * c;
  };
  
  Matrix4f matrix_dx;
  matrix_dx << sqaure_sum(points_[0](0), points_[0](1), points_[0](2)),
                  points_[0](1), points_[0](2), 1.0,
               sqaure_sum(points_[1](0), points_[1](1), points_[1](2)),
                  points_[1](1), points_[1](2), 1.0,
               sqaure_sum(points_[2](0), points_[2](1), points_[2](2)),
                  points_[2](1), points_[2](2), 1.0,
               sqaure_sum(points_[3](0), points_[3](1), points_[3](2)),
                  points_[3](1), points_[3](2), 1.0,
  double dx = matrix_dx.determinant();

  Matrix4f matrix_dy;
  matrix_dy << sqaure_sum(points_[0](0), points_[0](1), points_[0](2)),
                  points_[0](0), points_[0](2), 1.0,
               sqaure_sum(points_[1](0), points_[1](1), points_[1](2)),
                  points_[1](0), points_[1](2), 1.0,
               sqaure_sum(points_[2](0), points_[2](1), points_[2](2)),
                  points_[2](0), points_[2](2), 1.0,
               sqaure_sum(points_[3](0), points_[3](1), points_[3](2)),
                  points_[3](0), points_[3](2), 1.0,
  double dy = -matrix_dy.determinant();

  Matrix4f matrix_dz;
  matrix_dy << sqaure_sum(points_[0](0), points_[0](1), points_[0](2)),
                  points_[0](0), points_[0](1), 1.0,
               sqaure_sum(points_[1](0), points_[1](1), points_[1](2)),
                  points_[1](0), points_[1](1), 1.0,
               sqaure_sum(points_[2](0), points_[2](1), points_[2](2)),
                  points_[2](0), points_[2](1), 1.0,
               sqaure_sum(points_[3](0), points_[3](1), points_[3](2)),
                  points_[3](0), points_[3](1), 1.0,
  double dz = matrix_dz.determinant();

  sphere_center_ = Eigen::Vector3d(dx, dy, dz) * 0.5 / a;
  sphere_radius = (sphere_center_ - points_[0]).norm();
  double bias = sphere_radius - (sphere_center - points_[1]).norm();
  assert(std::abs(bias) < 1e-3);
  return;
}

bool Tetra::InSphere(Eigen Vector3d pt) {
  return ((pt - sphere_center_).norm() < sphere_radius_  - 1e-5);
};

void Tetrahedralization(const std::vector<Eigen::Vector3d>& points, std::vector<Tetra> *tetras) {
  const double inf = 1e7;
  Eigen::Vector3d lim_min( inf,  inf,  inf);
  Eigen::Vector3d lim_max(-inf, -inf, -inf);
  for (const auto &pt : points) {
    lim_min = Vector3d(
      std::min(lim_min(0), pt(0)),
      std::min(lim_min(1), pt(1)),
      std::min(lim_min(2), pt(2))
    );
    lim_max = Vector3d(
      std::max(lim_max(0), pt(0)),
      std::max(lim_max(1), pt(1)),
      std::max(lim_max(2), pt(2))
    )
  }
  lim_min -= Vector3d(10.0, 10.0, 10.0);
  lim_max += Vector3d(10.0, 10.0, 10.0);

  Eigen::Vector3d vec_x(lim_max(0) - lim_min(0), 0.0, 0.0);
  Eigen::Vector3d vec_y(0.0, lim_max(1) - lim_min(1), 0.0);
  Eigen::Vector3d vec_z(0.0, 0.0, lim_max(2) - lim_min(2));
  vec_x *= 5.0;
  vec_y *= 5.0;
  vec_z *= 5.0;
  
  std::list<Tetra> tetra_list;
  tetra_list.emplace_back(lim_min, lim_min + vec_x, lim_min + vec_y, lim_min + vec_z, true);
  for (const auto &pt : points) {
    auto cmp = [](const Trian &a, const Tiran &b){
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          if (std::abs(a.points_[i](j) - b.points_[i](j)) > 1e-5) {
            return a.points_[i](j) < b.points_[i](j);
          }
        }
      }
      return false;
    };
    
    std::set<Trian, decltype(cmp)> trians(cmp);
    for (auto iter = tetra_list.begin(); iter != tetra_list.end(); ) {
      if (iter->InSphere(pt)) {
        for (int i = 0; i < 4; i++)
          for (int j = i + 1; j < 4; j++)
            for (int k = j + 1; k < 4; k++) {
              Trian trian(iter->points_[i], iter->points_[j], iter->points_[k]);
              if (trians.find(trian) != trians.end()) {
                trians.erase(*trian);
              } else {
                trians.insert(*trian);
              }
            }
        iter = tetra_list.erase(iter);
      } else {
        iter++;
      }
    }
    
    for (const auto &trian : trians) {
      tetra_list.emplate_back(trian.points_[0], trian.points_[1], trian.points_[2], pt, true);
    }
  }

  // erase super tetra.
  for (auto iter = tetra_list.begin(); iter != tetra_list.end()) {
    bool exceed = false;
    for (int i = 0; i < 4 && !exceed; i++) {
      for (int j = 0; j < 3; j++) {
        if (iter->points_[i](j) < lim_min(j) + 1e-5 || iter->points_[i](j) > lim_max(j) - 1e-5) {
          exceed = true;
          break;
        }
      }
    }
    if (exceed) {
      iter = tetra_list.erase(iter);
    } else {
      iter++;
    }
  }

  tetras->resize(tetra_list.size());
  std::copy_n(tetra_list.begin(), tetra_list.size(), tetras->begin());
}


Trian::Trian(Eigen::Vector3d pt0, Eigen::Vector3d pt1, Eigen::Vector3d pt2) {
  points_[0] = pt0;
  points_[1] = pt1;
  points_[2] = pt2;
  auto cmp = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    for (int i = 0; i < 3; i++) {
      if (std::abs(a(i) - b(i)) > 1e-5) {
        return (a(i) < b(i));
      }
      return false;
    }
  };
  std::sort(points_, points_ + 3, cmp);
}