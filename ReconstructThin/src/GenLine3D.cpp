#include <line3D.h>
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// Eigen
#include <eigen3/Eigen/Eigen>
// std
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <list>

void AddImgs(std::string file_path, L3DPP::Line3D *line_3D) {
  int num_cam = 32;
  
  // Load Camera matrix.
  auto matrix_file_name = file_path + std::string("/Cameras.txt");
  FILE *f = fopen(matrix_file_name.c_str(), "r");
  std::vector<Eigen::Matrix3d> Ks, Rs;
  std::vector<Eigen::Vector3d> Ts;
  for (int i = 0; i < num_cam; i++) {
    // K, R, t
    Eigen::Matrix3d K, R;
    Eigen::Vector3d T;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double val;
        fscanf(f, "%lf", &val);
        K(i, j) = val;
      }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double val;
        fscanf(f, "%lf", &val);
        R(i, j) = val;
      }
    for (int i = 0; i < 3; i++) {
      double val;
      fscanf(f, "%lf", &val);
      T(i) = val;
    }
    Ks.push_back(K);
    Rs.push_back(R);
    Ts.push_back(T);
  }
  fclose(f);

  // load & add img
  for (int i = 0; i < num_cam; i++) {
    auto file_name = file_path + std::string("/") + std::to_string(i) + std::string(".png");
    cv::Mat img = cv::imread(file_name);
    auto neighbors = std::list<unsigned int>{
      static_cast<unsigned int>((i + 1) % num_cam), 
      static_cast<unsigned int>((i + num_cam - 1) % num_cam)
    };
    line_3D->addImage(
      i, img, Ks[i], Rs[i], Ts[i], 7, neighbors
    );
  }
}

int main() {
  auto line_3D = new L3DPP::Line3D(
    std::string("."),
    L3D_DEF_LOAD_AND_STORE_SEGMENTS,
    L3D_DEF_MAX_IMG_WIDTH,
    L3D_DEF_MAX_NUM_SEGMENTS,
    false,
    false // use_gpu
  );
  AddImgs(std::string("../data"), line_3D);
  line_3D->matchImages();
  line_3D->reconstruct3Dlines();
  line_3D->saveResultAsOBJ(std::string(""));
  return 0;
}