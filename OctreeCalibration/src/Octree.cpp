//
// Created by aska on 19-2-16.
//

#include "Octree.h"
#include "Utils.h"
#include <algorithm>
#include <fstream>

// TreeNode -------------------------------------------------------------------------------

double TreeNode::ToF(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  double min_t[3], max_t[3];
  // TODO: Hard code here.
  const double eps = 1e-8;
  const double inf = 1e10;
  for (int i = 0; i < 3; i++) {
    if (dir(i) < eps && dir(i) > -eps) {
      if (pos(i) > base_(i) && pos(i) < base_(i) + d_) {
        min_t[i] = -inf;
        max_t[i] = inf;
      } else {
        return -eps;
      }
    }
    else {
      min_t[i] = (base_(i) - pos(i)) / dir(i);
      max_t[i] = min_t[i] + d_ / dir(i);
      if (min_t[i] > max_t[i]) {
        std::swap(min_t[i], max_t[i]);
      }
    }
  }
  return std::min(std::min(max_t[0], max_t[1]), max_t[2]) - std::max(std::max(min_t[0], min_t[1]), min_t[2]);
}

bool TreeNode::Intersect(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  return ToF(pos, dir) > 0.0;
}

double TreeNode::CalcSingleForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, double fineness) {
  Eigen::Vector3d bias = (centroid_ / (double) num_points_) - pos;
  double len = bias.dot(dir);
  Eigen::Vector3d ort_vec = bias - len * dir;
  // TODO: Hard code here;
  // return 1.0 / (1.0 + std::exp(-ort_vec.norm() / fineness));
  double tmp = d_ / fineness;
  double dis = ort_vec.norm() / fineness;
  double density = (double) num_points_ / (tmp * tmp * tmp);
  return density > 1e-9 ? dis / (density * density): 1e10;
}

// Octree ---------------------------------------------------------------------------------

Octree::Octree(Eigen::Vector3d o, double r, double fineness): o_(o), r_(r), fineness_(fineness) {
  root_ = new TreeNode(Eigen::Vector3d(o) - Eigen::Vector3d(r, r, r),
    Eigen::Vector3d(0.0, 0.0, 0.0), r * 2.0, 0);
}


double Octree::CalcForce(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  ray_pos_ = pos;
  ray_dir_ = dir;
  return CalcForce(root_);
}

double Octree::CalcForce(TreeNode *node) {
  double tof = node->ToF(ray_pos_, ray_dir_);
  if (tof <= 0.0 || node->num_points_ == 0 || node->d_ < fineness_ + 1e-8) {
    return node->CalcSingleForce(ray_pos_, ray_dir_, fineness_);
  }
  else {
    double ret_force = 1e9;
    int idx = -1;
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        for (int k = 0; k < 2; k++) {
          ++idx;
          if (node->sons_[idx] == nullptr) {
            continue;
          }
          ret_force = std::min(ret_force, CalcForce(node->sons_[idx]));
        }
      }
    }
    return ret_force;
  }
}

void Octree::Add(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir) {
  ray_pos_ = pos;
  ray_dir_ = dir;
  if (!root_->Intersect(ray_pos_, ray_dir_)) {
    std::cout << "In Octree::Add" << std::endl;
    std::cout << "pos: " << ray_pos_.transpose() << " dir: " << ray_dir_.transpose() << std::endl;
    std::cout << "fuck" << std::endl;
    exit(0);
  }
  Add(root_);
}

int Octree::Add(TreeNode *node) {
  if (node->d_  < fineness_ + 1e-9) {
    node->num_points_++;
    node->centroid_ += node->base_ + Eigen::Vector3d(node->d_, node->d_, node->d_) * 0.5;
    return 1;
  }
  int idx = -1;
  int added = 0;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++) {
        ++idx;
        if (node->sons_[idx] == nullptr) {
          node->sons_[idx] = new TreeNode(node->base_ + Eigen::Vector3d(i, j, k) * 0.5 * node->d_,
            Eigen::Vector3d(0.0, 0.0, 0.0), node->d_ * 0.5, 0);
        }
        if (!node->sons_[idx]->Intersect(ray_pos_, ray_dir_)) {
          continue;
        }
        node->centroid_ -= node->sons_[idx]->centroid_;
        added += Add(node->sons_[idx]);
        node->centroid_ += node->sons_[idx]->centroid_;
      }
    }
  }
  node->num_points_ += added;
  return added;
}

void Octree::OutputMesh(std::string mesh_path, double fineness, double density) {
  std::vector<TreeNode *> out_cubes;
  out_cubes.clear();
  FindDenseCubes(root_, fineness, density, out_cubes);
  //out_cubes.push_back(new TreeNode(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 1.0, 0));
  //out_cubes.push_back(new TreeNode(Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 1.0, 0));
  //out_cubes.push_back(new TreeNode(Eigen::Vector3d(0.0, 2.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 1.0, 0));
  //out_cubes.push_back(new TreeNode(Eigen::Vector3d(0.0, 0.0, 2.0), Eigen::Vector3d(0.0, 0.0, 0.0), 1.0, 0));
  std::vector<Trian> trians;
  for (auto cube : out_cubes) {
    for (int axis = 0; axis < 3; axis++) {
      for (int dir = 0; dir < 2; dir++) {
        Eigen::Vector3d pt(cube->base_);
        pt += Eigen::Vector3d(cube->d_, cube->d_, cube->d_) * 0.5;
        pt(axis) += (dir - 0.5) * cube->d_;
        Eigen::Vector3d vec(cube->base_);
        vec(axis) = pt(axis);
        Eigen::Matrix3d t;
        Eigen::Vector3d rot_vec(pt - (cube->base_ + Eigen::Vector3d(cube->d_, cube->d_, cube->d_) * 0.5));
        rot_vec /= rot_vec.norm();
        t = Eigen::AngleAxisd(0.5 * M_PI, rot_vec);
        for (int i = 0; i < 4; i++) {
          Eigen::VectorXd new_vec;
          new_vec = t * (vec - pt) + pt;
          trians.emplace_back(pt, vec, new_vec, false);
          vec = new_vec;
        }
      }
    }
  }
  Utils::SaveTriansAsPly(mesh_path, trians);
}

void Octree::FindDenseCubes(TreeNode* node, double fineness, double density, std::vector<TreeNode *> &out_cubes) {
  if (fineness > node->d_- 1e-8 && fineness <= node->d_ * 2.0) {
    double tmp = node->d_ / fineness;
    if (node->num_points_ / (tmp * tmp * tmp) > density) {
      out_cubes.push_back(node);
      // std::cout << node->num_points_ / (tmp * tmp * tmp) << std::endl;
    }
    return;
  }
  for (int i = 0; i < 8; i++) {
    if (node->sons_[i] != nullptr) {
      FindDenseCubes(node->sons_[i], fineness, density, out_cubes);
    }
  }
}