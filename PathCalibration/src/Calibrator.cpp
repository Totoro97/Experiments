//
// Created by aska on 19-1-29.
//

#include "Calibrator.h"

void Calibrator::Run() {
  // Initialize Parameters.

  int num_ex_paras_ = 0;
  past_sampled_.resize(path_2d_.size(), -1);
  next_sampled_.resize(path_2d_.size(), -1);
  // TODO: Hard code here.
  depth_.resize(path_2d_.size(), 2.3);

  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }

    if (i == 0 || i == path_2d_.size() - 1 || path_2d_[i - 1](0) == -1 || path_2d_[i + 1](0) == -1) {
      past_sampled_[i] = next_sampled_[i] = i;
      sampled_.push_back(i);
      num_ex_paras_++;
    }
    else if (i % 40 == 0) {
      past_sampled_[i] = next_sampled_[i] = i;
      sampled_.push_back(i);
      num_ex_paras_++;
    }
  }
  std::cout << "num_ex_paras = " << num_ex_paras_ << std::endl;

  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) != -1 && past_sampled_[i] == -1) {
      past_sampled_[i] = past_sampled_[i - 1];
    }
  }
  for (int i = (int) path_2d_.size() - 1; i >= 0; i--) {
    if (path_2d_[i](0) != -1 && next_sampled_[i] == -1) {
      next_sampled_[i] = next_sampled_[i + 1];
    }
  }

  int num_valid_funcs = 0;
  for (const auto &pt : path_2d_) {
    if (pt(0) != -1) {
      num_valid_funcs++;
    }
  }

  // ShowSampledPoints();
  SaveCurrentPoints();
  ShowCurrentSituation();
  int iter_counter = 0;
  while (true) {
    iter_counter++;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_valid_funcs, 7 + num_ex_paras_);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(num_valid_funcs);
    int idx = -1;
    for (int i = 0; i < path_2d_.size(); i++) {
      if (path_2d_[i](0) == -1) {
        continue;
      }
      idx++;
      auto warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
      B(idx) = dist_map_->Distance(warped(0), warped(1));
      const double step_len = 1.0 / (double) (1 << 10);
      Eigen::Vector2d grad_i;
      grad_i(0) =
        (dist_map_->Distance(warped(0) + step_len, warped(1)) - B(idx)) / step_len;
      grad_i(1) =
        (dist_map_->Distance(warped(0), warped(1) + step_len) - B(idx)) / step_len;
      // Depth Paras.
      int p_idx = -1;
      for (int p : sampled_) {
        p_idx++;
        // fix first depth
        if (p_idx == 0 || (iter_counter & 3) != 0) {
          continue;
        }
        if (next_sampled_[i] != p && past_sampled_[i] != p) {
          A(idx, p_idx) = 0.0;
          continue;
        }
        double new_depth_ = depth_[p] + step_len;
        std::swap(new_depth_, depth_[p]);
        auto new_warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
        std::swap(new_depth_, depth_[p]);
        A(idx, p_idx) = grad_i.dot((new_warped - warped) / step_len);
      }

      // Camera Paras.
      for (int t = 0; t < 7; t++) {
        p_idx++;
        if (t < 3 && (iter_counter & 3) != 2) {
          continue;
        }
        if (t >= 3 && t < 6 && (iter_counter & 3) != 1) {
          continue;
        }
        if (t >= 6 && (iter_counter & 3) != 3) {
          continue;
        }
        cam_paras_[t] += step_len;
        auto new_warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
        cam_paras_[t] -= step_len;
        A(idx, p_idx) = grad_i.dot((new_warped - warped) / step_len);
        //if (t == 6) {
        //  std::cout << "focus length gradient: " << A(idx, p_idx) << std::endl;
        //}
      }
    }

    // Drop out.
    // TODO: Hard code here.
    double drop_out_ratio = -1.0;
    double all_sum = 0.0;
    std::vector<std::pair<double, int> > rank_list;
    for (int j = 0; j < 7 + num_ex_paras_; j++) {
      double tmp_sum = 0.0;
      for (int i = 0; i < num_valid_funcs; i++) {
        tmp_sum += std::abs(A(i, j));
      }
      rank_list.emplace_back(tmp_sum, j);
      all_sum += tmp_sum;
    }
    std::sort(rank_list.begin(), rank_list.end());
    double res = all_sum * drop_out_ratio;
    for (auto iter = rank_list.begin(); iter != rank_list.end() && res > 0.0; iter++) {
      res -= iter->first;
      if (res > 1e-9) {
        int idx_p = iter->second;
        for (int idx_i = 0; idx_i < num_valid_funcs; idx_i++) {
          A(idx_i, idx_p) = 0.0;
        }
      }
    }
    // Solve least square.
    // Eigen::VectorXd delta_p = A.colPivHouseholderQr().solve(-B);
    // ---------------------
    Eigen::VectorXd delta_p = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-B);
    // ---------------------
    // Or: Grad?
    // Eigen::VectorXd delta_p = Eigen::VectorXd::Zero(7 + num_ex_paras_);
    // for (int i = 0; i < num_valid_funcs; i++) {
    //   delta_p += A.block(i, 0, 1, 7 + num_ex_paras_).transpose();
    // }
    // delta_p *= -1e-3 / num_valid_funcs;

    res = all_sum * drop_out_ratio;
    for (auto iter = rank_list.begin(); iter != rank_list.end() && res > 0.0; iter++) {
      res -= iter->first;
      if (res > 1e-9) {
        int idx_p = iter->second;
        delta_p(idx_p) = 0.0;
      }
    }

    double current_error = CalcCurrentError();
    while (true) {
      std::cout << delta_p << std::endl;
      AddDeltaP(delta_p);
      double new_error = CalcCurrentError();
      SaveCurrentPoints();
      ShowCurrentSituation();
      /*if (new_error > current_error * 1.1 || delta_p.norm() > 1.0 ) {
        AddDeltaP(-delta_p);
        delta_p *= 0.5;
      }
      else {*/
        break;
      //}
    }
    /*if (new_error > current_error) {
      AddDeltaP(-delta_p);
      break;
    };*/
  }
}

void Calibrator::AddDeltaP(const Eigen::VectorXd &delta_p) {
  int p_idx = -1;
  for (int p : sampled_) {
    depth_[p] += delta_p(++p_idx);
  }
  for (int t = 0; t < 7; t++) {
    cam_paras_[t] += delta_p(++p_idx);
  }
}

void Calibrator::ShowSampledPoints() {
  cv::Mat img(dist_map_->height_, dist_map_->width_, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < path_2d_.size(); i++) {
    if (next_sampled_[i] == i) {
      cv::circle(img, cv::Point(path_2d_[i](1), path_2d_[i](0)), 0, cv::Scalar(0, 255, 255), 1);
    }
  }
  cv::imshow("Sampled", img);
  cv::waitKey(-1);
}

double Calibrator::CalcCurrentError() {
  int idx = -1;
  double current_error = 0;
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    idx++;
    auto warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
    current_error += dist_map_->Distance(warped(0), warped(1));
  }
  current_error /= (double) idx;
  return current_error;
}

void Calibrator::ShowCurrentSituation() {
  int idx = -1;
  cv::Mat img;
  cv::cvtColor(img_gray_, img, cv::COLOR_GRAY2BGR);
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    idx++;
    auto warped = Warp(path_2d_[i](0), path_2d_[i](1), GetDepth(i));
    cv::circle(img, cv::Point(warped(1), warped(0)), 0, cv::Scalar(0, 255, 255), 1);
  }
  std::cout << "current_error: " << CalcCurrentError() << std::endl;
  cv::imwrite(std::string("current_") + std::to_string(sit_counter_++) + std::string(".png"), img);
  cv::imshow("Current", img);
  cv::waitKey(-1);
}

void Calibrator::SaveCurrentPoints() {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < path_2d_.size(); i++) {
    if (path_2d_[i](0) == -1) {
      continue;
    }
    // TODO: Hard code here.
    double focal_length = std::exp(cam_paras_[6]);
    double z = GetDepth(i);
    double x = (path_2d_[i](1) - dist_map_->width_ / 2.0) * focal_length * z;
    double y = (path_2d_[i](0) - dist_map_->height_ / 2.0) * focal_length * z;
    points.emplace_back(z, x, y);
  }
  Utils::SavePointsAsPly("current.ply", points);
}


double Calibrator::GetDepth(int idx) {
  double depth;
  if (next_sampled_[idx] == idx) {
    depth = depth_[idx];
  }
  else {
    int a = past_sampled_[idx];
    int b = next_sampled_[idx];
    double bias = ((double) (idx - a)) / ((double) (b - a));
    depth = depth_[a] * (1.0 - bias) + depth_[b] * bias;
  }
  return std::exp(depth);
}

Eigen::Vector2d Calibrator::Warp(int i, int j, double depth) {
  double focal_length = std::exp(cam_paras_[6]);
  // double focal_length = cam_paras_[6];
  // double focal_length = 1.0;

  double x = (j - dist_map_->width_  / 2.0) * focal_length * depth;
  double y = (i - dist_map_->height_ / 2.0) * focal_length * depth;
  double z = depth;
  // 0, 1, 2: translation.
  double t_x = x + cam_paras_[0];
  double t_y = y + cam_paras_[1];
  double t_z = z + cam_paras_[2];
  // 3, 4, 5: rotation.
  Eigen::Vector3d w(cam_paras_[3], cam_paras_[4], cam_paras_[5]);
  Eigen::Matrix3d t;
  if (w.norm() < 1e-6) {
    t.setIdentity();
  }
  else {
    t = Eigen::AngleAxisd(w.norm(), w / w.norm());
  }
  auto r_coord = t * Eigen::Vector3d(t_x, t_y, t_z);

  // std::cout << "z = " << r_coord(2) << std::endl;
  return Eigen::Vector2d(
    r_coord(1) / (r_coord(2) * focal_length) + (double) dist_map_->height_ / 2,
    r_coord(0) / (r_coord(2) * focal_length) + (double) dist_map_->width_ / 2
    );
}