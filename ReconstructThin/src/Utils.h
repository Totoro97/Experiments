#pragma once
// Eigen
#include <eigen3/Eigen/Eigen>
// std
#include <string>
#include <vector>
// me
#include "Tetra.h"

namespace Utils {

void ReadKRTFromFile(std::string file_path, int cam_num,
                     std::vector<Eigen::Matrix3d> *Ks,
                     std::vector<Eigen::Matrix3d> *Rs,
                     std::vector<Eigen::Vector3d> *Ts);

void SaveTriansAsPly(std::string save_path, const std::vector<Trian> &trians);

void Srand();
double Random();
double RandomLR(double l, double r);
}
