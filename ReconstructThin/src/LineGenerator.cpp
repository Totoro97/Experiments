#include "LineGenerator.h"

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// std
#include <cstdio>
#include <iostream>

void LineGenerator::AddImgs(std::string file_path, L3DPP::Line3D *line_3D) {
  int num_cam = 64;
  
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
    std::list<unsigned int> neighbors;
    for (int j = -5; j <= 5; j++) {
      neighbors.push_back((i + j + num_cam) % num_cam);
    }
    //auto neighbors = std::list<unsigned int>{1};
    line_3D->addImage(
      i, img, Ks[i], Rs[i], Ts[i], 1e9, neighbors
    );
  }
}

void LineGenerator::GenerateLine() {
  line_3D_ = new L3DPP::Line3D(
    std::string("."),
    L3D_DEF_LOAD_AND_STORE_SEGMENTS,
    L3D_DEF_MAX_IMG_WIDTH,
    L3D_DEF_MAX_NUM_SEGMENTS,
    false,
    false // use_gpu
  );
  AddImgs(work_path_, line_3D_);
  line_3D_->matchImages();
  line_3D_->reconstruct3Dlines();
}


void LineGenerator::GetSegment3D(
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* segments) {
  std::cout << "GetSegment3D: Begin" << std::endl;
  std::vector<L3DPP::FinalLine3D> result;
  line_3D_ -> get3Dlines(result);
  std::cout << "get3Dlines: End" << std::endl;
  std::cout << "result's size = " << result.size() << std::endl;
  for (const L3DPP::FinalLine3D &line : result) {
    auto segment = line.underlyingCluster_.seg3D();
    segments->push_back(std::make_pair(segment.P1(), segment.P2()));
  }
  std::cout << "GetSegment3D:: End" << std::endl;
  return;
}