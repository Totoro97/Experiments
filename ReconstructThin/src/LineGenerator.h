#pragma once
#include <line3D.h>
// Eigen
#include <eigen3/Eigen/Eigen>
// std
#include <string>
#include <vector>
#include <list>

class LineGenerator {

public:
  LineGenerator(std::string work_path, int cam_num) {
    work_path_ = work_path;
    cam_num_ = cam_num;
  }

  ~LineGenerator() {
    delete(line_3D_);
  }

  void AddImgs(std::string file_path, L3DPP::Line3D *line_3D);
  void GenerateLine();
  void GetSegment3D(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* segments);

  std::vector<Eigen::Matrix3d> Ks_, Rs_;
  std::vector<Eigen::Vector3d> Ts_;
  L3DPP::Line3D *line_3D_;
  int cam_num_;
  std::string work_path_;
};