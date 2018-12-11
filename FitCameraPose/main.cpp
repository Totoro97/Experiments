#include "Headers.h"
#include "Utils.h"
#include "View.h"
// std
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

void ViewAdjust(View *view_0, View *view_1, std::vector<Eigen::Vector3d> &points) {
  std::vector<double> grads;
  grads.resize(points.size());
  double point_step_length = 1e-2;
  double cam_pos_step_length = 1e-2;
  double cam_rot_step_length = 1e-2;
  double energy = 0.5;
  double ratio = 0.995;
  Eigen::Matrix4d world_to_view_0;
  world_to_view_0.block(0, 0, 3, 3) = view_0->R_;
  world_to_view_0.block(0, 3, 3, 1) = view_0->T_;
  world_to_view_0.block(3, 0, 1, 4) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0).transpose();
  Eigen::Matrix4d view_0_to_world = world_to_view_0.inverse();
  Eigen::Vector3d view_0_pos = view_0_to_world.block(0, 3, 3, 1);

  while (energy > 1e-4) {
    std::cout << "energy = " << energy << std::endl;
    // Adjust Camera Pose
    for (int num_iter = 10; num_iter; num_iter--) {
      Eigen::Vector3d bias(Utils::RandomLR(-cam_pos_step_length, cam_pos_step_length),
                           Utils::RandomLR(-cam_pos_step_length, cam_pos_step_length),
                           Utils::RandomLR(-cam_pos_step_length, cam_pos_step_length));
      double past_cost = 0.0, new_cost = 0.0;
      for (auto iter = points.begin(); iter != points.end(); iter++) {
        past_cost += view_1->Distance(*iter);
      }
      std::cout << "past_cost = " << past_cost << std::endl;
      view_1->T_ += bias;
      for (auto iter = points.begin(); iter != points.end(); iter++) {
        new_cost += view_1->Distance(*iter);
      }
      if (new_cost > past_cost /*&& Utils::Random() > energy*/) {
        view_1->T_ -= bias;
      } else {
        past_cost = new_cost;
      }

      double a = Utils::RandomLR(-cam_rot_step_length, cam_rot_step_length);
      double b = Utils::RandomLR(-cam_rot_step_length, cam_rot_step_length);
      double c = Utils::RandomLR(-cam_rot_step_length, cam_rot_step_length);

      Eigen::Matrix3d A, B, C;
      A << std::cos(a), std::sin(a), 0.0,
              -std::sin(a), std::cos(a), 0.0,
              0.0, 0.0, 1.0;
      /*B << 1.0, 0.0, 0.0,
              0.0, std::cos(b), std::sin(b),
              0.0, -std::sin(b), std::cos(b);
      C << std::cos(c), std::sin(c), 0.0,
              -std::sin(c), std::cos(c), 0.0,
              0.0, 0.0, 1.0;
      */
      Eigen::Matrix3d rot = A;
      new_cost = 0.0;
      auto past_R = view_1->R_;
      view_1->R_ *= rot;
      for (auto iter = points.begin(); iter != points.end(); iter++) {
        new_cost += view_1->Distance(*iter);
      }
      if (new_cost > past_cost /*&& Utils::Random() > energy*/) {
        view_1->R_ = past_R;
      } else {
        past_cost = new_cost;
      }
    }
    // Adjust Point

    for (auto iter = points.begin(); iter != points.end(); iter++) {
      Eigen::Vector3d pt = *iter;
      double bias = Utils::RandomLR(-point_step_length, point_step_length);
      Eigen::Vector3d vec = pt - view_0_pos;
      vec = vec / vec.norm() * bias;
      auto new_pt = pt + vec;
      bool is_better = view_1->Distance(new_pt) < view_1->Distance(pt);
      if (is_better || Utils::Random() < energy) {
        *iter = new_pt;
      }
    }

    // Ratio
    point_step_length *= ratio;
    cam_pos_step_length *= ratio;
    cam_rot_step_length *= ratio;
    energy *= ratio;
  }
}

int main() {
  Eigen::Vector3d x(1, 2, 3);
  double fl = 1050.0;
  int num_view = 2;
  std::vector<View *> views;
  for (int i = 0; i < num_view; i++) {
    auto tmp_ptr = new View(
            std::string("C:/Users/Aska/tmp/") + std::to_string(i) + std::string(".png"),
            false, std::to_string(i) + std::string(".bin"));
    views.push_back(tmp_ptr);
  }
  if (views.size() < 2) {
    std::cout << "Not enough view, return;" << std::endl;
  }

  Eigen::Matrix3d K_init, R_init;
  Eigen::Vector3d T_init(0.0, 0.0, 0.0);
  K_init << fl, 0.0, views[0]->width_ * 0.5,
            0.0, fl, views[0]->height_ * 0.5,
            0.0, 0.0, 1.0;
  R_init << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
  views[0]->SetKRT(K_init, R_init, T_init);

  std::vector<Eigen::Vector3d> points;

  for (auto view_1 = std::next(views.begin()); view_1 != views.end(); view_1++) {
    std::cout << "fuck" << std::endl;
    auto view_0 = std::prev(view_1);
    (*view_1)->SetKRT((*view_0)->K_, (*view_0)->R_, (*view_0)->T_);
    (*view_0)->SamplePoints(points, 0.1);
    std::cout << "points size: " << points.size() << std::endl;
    ViewAdjust(*view_0, *view_1, points);
  }
  Utils::SavePointCloudAsPly(points, std::string("points.ply"));
  return 0;
}