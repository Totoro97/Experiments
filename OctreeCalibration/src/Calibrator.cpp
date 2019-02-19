//
// Created by aska on 19-1-29.
//

#include "Calibrator.h"
#include "Utils.h"

Calibrator::Calibrator(const cv::Mat &img_gray, Octree* octree, int frame_id, double ang) :
  img_gray_(img_gray), octree_(octree), frame_id_(frame_id) {
  height_ = img_gray.rows;
  width_ = img_gray.cols;

  // TODO: Hard code here.
  pos_ = Eigen::Vector3d(10.0 * std::cos(ang), 10.0 * std::sin(ang), 0.0) +
    Eigen::Vector3d(Utils::RandomLR(-0.1, 0.1), Utils::RandomLR(-0.1, 0.1), Utils::RandomLR(-0.1, 0.1));
  x_axis_ = Eigen::Vector3d(-std::sin(ang), std::cos(ang), 0.0) +
    Eigen::Vector3d(Utils::RandomLR(-0.1, 0.1), Utils::RandomLR(-0.0, 0.0), Utils::RandomLR(-0.0, 0.0));
  y_axis_ = Eigen::Vector3d(0.0, 0.0, -1.0) +
    Eigen::Vector3d(Utils::RandomLR(-0.1, 0.1), Utils::RandomLR(-0.0, 0.0), Utils::RandomLR(-0.0, 0.0));
  x_axis_ /= x_axis_.norm();
  y_axis_ /= y_axis_.norm();
  z_axis_ = x_axis_.cross(y_axis_);

  // TODO: Need Sampling?
  int cnt = 0;
  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < width_; j++) {
      if (img_gray_.data[i * width_ + j] == 255 && ++cnt % 1 == 0) {
        visible_pixs_.emplace_back(j - width_ * 0.5, i - height_ * 0.5);
      }
    }
  }
}

double Calibrator::CalcCurrentF() {
  double current_f = 0.0;
  for (const auto &pix : visible_pixs_) {
    Eigen::Vector3d dir = pix(0) * x_axis_ + pix(1) * y_axis_ + scale_ratio_ * z_axis_;
    dir /= dir.norm();
    current_f += octree_->CalcForce(pos_, dir);
  }
  return current_f;
}

void Calibrator::Calibrate() {
  double go_len_trans = 0.1;
  double go_len_rot = 0.01;
  double temper = 0.0;
  int iter_counter = 0;
  std::cout << CalcCurrentF() << std::endl;
  for (; iter_counter < 200;) {
    iter_counter++;
    go_len_trans *= 0.995;
    go_len_rot *= 0.995;
    temper *= 0.985;
    double current_f = CalcCurrentF();

    auto current_pos = pos_;
    auto current_x_axis = x_axis_;
    auto current_y_axis = y_axis_;
    auto current_z_axis = z_axis_;

    Eigen::Vector3d trans(0.0, 0.0, 0.0);
    Eigen::Vector3d rot(0.0, 0.0, 0.0);

    if (iter_counter % 2 == 0) {
      trans = Eigen::Vector3d(
        Utils::RandomLR(-go_len_trans, go_len_trans),
        Utils::RandomLR(-go_len_trans, go_len_trans),
        Utils::RandomLR(-go_len_trans, go_len_trans));
    }
    else {
      rot = Eigen::Vector3d(
        Utils::RandomLR(-go_len_rot, go_len_rot),
        Utils::RandomLR(-go_len_rot, go_len_rot),
        Utils::RandomLR(-go_len_rot, go_len_rot));
    }

    pos_ = current_pos + trans;

    Eigen::Matrix3d t;
    if (rot.norm() > 1e-7) {
      t = Eigen::AngleAxisd(rot.norm(), rot / rot.norm());
    }
    else {
      t.setIdentity();
    }
    x_axis_ = t * current_x_axis;
    y_axis_ = t * current_y_axis;
    x_axis_ /= x_axis_.norm();
    y_axis_ = y_axis_ - x_axis_ * x_axis_.dot(y_axis_);
    y_axis_ /= y_axis_.norm();
    z_axis_ = x_axis_.cross(y_axis_);
    double new_f = CalcCurrentF();
    std::cout << frame_id_ << " " << iter_counter << " " << go_len_rot << " " << new_f << " " << current_f << std::endl;
    std::cout << "temper = " << temper << std::endl;
    if (new_f > current_f && Utils::Random() > temper) {
      pos_ = current_pos;
      x_axis_ = current_x_axis;
      y_axis_ = current_y_axis;
      z_axis_ = current_z_axis;
    }
  }
  std::cout << "pos = " << pos_.transpose() << std::endl;
  // Update
}

void Calibrator::AddToOctree() {
  for (const auto &pix : visible_pixs_) {
    Eigen::Vector3d dir = pix(0) * x_axis_ + pix(1) * y_axis_ + scale_ratio_ * z_axis_;
    dir /= dir.norm();
    octree_->Add(pos_, dir);
  }
}

void Calibrator::CopyCamParasFrom(Calibrator *another_calibrator) {
  pos_ = another_calibrator->pos_;
  x_axis_ = another_calibrator->x_axis_;
  y_axis_ = another_calibrator->y_axis_;
  z_axis_ = another_calibrator->z_axis_;
}