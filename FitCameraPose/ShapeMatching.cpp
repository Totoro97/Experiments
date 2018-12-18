#include "Headers.h"
#include "Utils.h"
#include "View.h"
// std
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

typedef Eigen::Matrix<double, 96, 1> ShapeContext;

typedef std::vector<ShapeContext> Histos;

bool HavePath(int i, int nl, int nr, double *edge,
        std::vector<double> &slk, std::vector<int> &vis_l,
        std::vector<int> &vis_r, std::vector<double> &mark_l, std::vector<double> &mark_r,
        std::vector<int> &p) {
  vis_l[i] = 1;
  for (int j = 0; j < nr; j++) {
    if (std::abs(edge[i * nr + j] - mark_l[i] - mark_r[j]) < 1e-3) {
      if (vis_r[j])
        continue;
      vis_r[j] = 1;
      if (!p[j] || HavePath(p[j], nl, nr, edge, slk, vis_l, vis_r, mark_l, mark_r, p)) {
        p[j] = i;
        std::cout << "Yes " << i << " " << j << std::endl;
        return true;
      }
    }
    else {
      slk[j] = std::min(slk[j], edge[i * nr + j] - mark_l[i] - mark_r[j]);
    }
  }
  return false;
}

void KM(const Histos &histos_0, const Histos &histos_1,
        const std::vector<Eigen::Vector2d> &points_0, const std::vector<Eigen::Vector2d> &points_1,
        std::vector<int> &p) {
    int nl = histos_0.size();
    int nr = histos_1.size();
    p.resize(nr, 0);
  std::vector<double> mark_l, mark_r;
  for (int i = 0; i < nl; i++)
    mark_l.push_back(1e9);
  for (int i = 0; i < nr; i++)
    mark_r.push_back(0);
  double* edge = new double[nl * nr];
  for (int i = 0; i < nl; i++)
    for (int j = 0; j < nr; j++) {
      edge[i * nr + j] = ((histos_0[i] - histos_1[j]).norm()) + (points_0[i] - points_1[j]).norm() * 0;
      // astd::cout << edge[i * nr + j] << std::endl;
    }
  for (int i = 0; i < nl; i++) {
    for (int j = 0; j < nr; j++) {
      mark_l[i] = std::min(mark_l[i], edge[i * nr + j]);
    }
  }

  std::vector<double> slk;
  slk.resize(nr);
  std::vector<int> vis_l, vis_r;
  vis_l.resize(nl, 0);
  vis_r.resize(nr, 0);
  for (int i = 0; i < nl; i++) {
    for (int j = 0; j < nr; j++)
      slk[j] = 1e9;
    for (int j = 0; j < nl; j++)
      vis_l[j] = 0;
    for (int j = 0; j < nr; j++)
      vis_r[j] = 0;

    while (!HavePath(i, nl, nr, edge, slk, vis_l, vis_r, mark_l, mark_r, p)) {
      double tmp = 1e9;
      for (int j = 0; j < nr; j++) {
        if (!vis_r[j])
          tmp = std::min(tmp, slk[j]);
      }
      for (int j = 0; j < nl; j++) {
        if (vis_l[j])
          mark_l[j] += tmp;
      }
      for (int j = 0; j < nr; j++) {
        if (vis_r[j])
          mark_r[j] -= tmp;
      }
      for (int j = 0; j < nr; j++) {
        slk[j] = 1e9;
        vis_r[j] = 0;
      }
    }
  }
  delete[](edge);
}

int main() {
  std::ofstream match_stream;
  match_stream.open("C:/Users/Aska/tmp/jpg/matched.txt", std::ios::ate);

  for (int m_ = 0; m_ < 64; m_++) {
    for (int n_ = m_ + 1; n_ <= m_ + 2; n_++) {
      int m = m_;
      int n = (n_ + 64) % 64;
      auto img_0 = cv::imread("C:/Users/Aska/tmp/" + std::to_string(m) + std::string(".png"));
      auto img_1 = cv::imread("C:/Users/Aska/tmp/" + std::to_string(n) + std::string(".png"));

      match_stream << std::to_string(m) + std::string(".jpg") << " " << std::to_string(n) + std::string(".jpg");
      auto IsInside = [](cv::Mat &img, int i, int j) {
        int width_ = img.cols;
        uint8_t *img_ptr = (uint8_t *) img.data;
        return (static_cast<int>(img_ptr[(i * width_ + j) * 3]) != img_ptr[0] ||
                static_cast<int>(img_ptr[(i * width_ + j) * 3 + 1]) != img_ptr[1] ||
                static_cast<int>(img_ptr[(i * width_ + j) * 3 + 2]) != img_ptr[2]);
      };

      int height = img_0.rows;
      int width = img_0.cols;
      std::vector<Eigen::Vector2d> points_0, points_1;
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          if (IsInside(img_0, i, j))
            points_0.emplace_back(static_cast<double>(i), static_cast<double>(j));
          if (IsInside(img_1, i, j))
            points_1.emplace_back(static_cast<double>(i), static_cast<double>(j));
        }
      }

      std::cout << "img_0: size = " << points_0.size()
                << " img_1: size = " << points_1.size() << std::endl;
      int sampled_size = 1000;
      std::srand(1233);

      for (int i = 0; i < sampled_size; i++)
        points_0[i] = points_0[points_0.size() / sampled_size * i];
      for (int i = 0; i < sampled_size; i++)
        points_1[i] = points_1[points_1.size() / sampled_size * i];
      // std::random_shuffle(points_0.begin(), points_0.end());
      // std::copy_n(points_0.begin(), points_0.size(), points_1.begin());
      // std::random_shuffle(points_1.begin(), points_1.end());

      std::ofstream out_stream;
      out_stream.open(std::string("C:/Users/Aska/tmp/jpg/") + std::to_string(m) + ".sift", std::ios::ate);
      out_stream << sampled_size << " " << "128\n";
      for (int i = 0; i < points_0.size(); i++) {
        out_stream << points_0[i](1) << " " << points_0[i](0) << " " << "2.19258 5.312\n";
        for (int j = 0; j < 128; j++)
          out_stream << " 0";
        out_stream << "\n";
      }
      out_stream.close();
      std::vector<ShapeContext> histos_0, histos_1;
      const double pi = std::acos(-1.0);
      for (int i = 0; i < sampled_size; i++) {
        ShapeContext histo;
        for (int i = 0; i < 96; i++) histo(i) = 0.0;
        for (int j = 0; j < sampled_size; j++) {
          auto v = (points_0[i] - points_0[j]);
          int dis = static_cast<int>(std::log2(1.0 + v.norm()));
          if (dis >= 8) continue;
          int block = static_cast<int>((std::atan2(v(1), v(0)) / (2.0 * pi) + 0.5 + 1e-4) * 12);
          block = std::min(block, 11);
          histo(block * 8 + dis) = histo(block * 8 + dis) + 1.0;
        }
        histos_0.push_back(histo);
      }
      for (int i = 0; i < sampled_size; i++) {
        ShapeContext histo;
        for (int i = 0; i < 96; i++) histo(i) = 0.0;
        for (int j = 0; j < sampled_size; j++) {
          auto v = (points_1[i] - points_1[j]);
          int dis = static_cast<int>(std::log2(1.0 + v.norm()));
          if (dis >= 8) continue;
          int block = static_cast<int>((std::atan2(v(1), v(0)) / (2.0 * pi) + 0.5 + 1e-4) * 12);
          block = std::min(block, 11);
          histo(block * 8 + dis) = histo(block * 8 + dis) + 1.0;
        }
        histos_1.push_back(histo);
      }

      std::vector<int> p;
      KM(histos_0, histos_1, points_0, points_1, p);

      /* cv::Mat pre_image(height, width * 2, CV_8UC3);
      uint8_t *pre_ptr = pre_image.data;
      uint8_t *l_ptr = img_0.data;
      uint8_t *r_ptr = img_1.data;
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
          for (int k = 0; k < 3; k++) {
            pre_ptr[(i * width * 2 + j) * 3 + k] = l_ptr[(i * width + j) * 3 + k];
            pre_ptr[(i * width * 2 + j + width) * 3 + k] = r_ptr[(i * width + j) * 3 + k];
          }
        }
      }
      for (int j = 0; j < sampled_size; j++) {
        int i = p[j];
        if ((histos_0[i] - histos_1[j]).norm() > 25)
          continue;
        std::cout << i << " " << j << std::endl;
        cv::line(pre_image, x, y, cv::Scalar(255, 255, 255), 1);
      } */
      cv::Mat pre_image(height, width, CV_8UC3);
      for (int i = 0; i < sampled_size; i++) {
        cv::circle(pre_image, cv::Point(points_0[i](1), points_0[i](0)), 1, cv::Scalar(255, 0, 0));
        cv::circle(pre_image, cv::Point(points_1[i](1), points_1[i](0)), 1, cv::Scalar(0, 255, 0));
      }
      std::vector<std::pair<int, int>> matched;
      matched.clear();
      for (int j = 0; j < sampled_size; j++) {
        int i = p[j];
        if ((histos_0[i] - histos_1[j]).norm() > 40)
          continue;
        cv::Point x(points_0[i](1), points_0[i](0));
        cv::Point y(points_1[j](1), points_1[j](0));
        cv::line(pre_image, x, y, cv::Scalar(255, 255, 255), 1);
        matched.emplace_back(i, j);
      }
      match_stream << " " << matched.size() << std::endl;
      for (auto iter = matched.begin(); iter != matched.end(); iter++)
        match_stream << iter->first << " ";
      match_stream << "\n";
      for (auto iter = matched.begin(); iter != matched.end(); iter++)
        match_stream << iter->second << " ";
      match_stream << "\n";
      cv::imwrite("C:/Users/Aska/tmp/matched" + std::to_string(m) + "_" + std::to_string(n) + ".png", pre_image);
    }
  }
  match_stream.close();
  return 0;
}