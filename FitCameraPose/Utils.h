#pragma once
#include "Headers.h"
// std
#include <string>
#include <vector>
// me

namespace Utils {

void ReadKRTFromFile(std::string file_path, int cam_num,
                     std::vector<Eigen::Matrix3d> *Ks,
                     std::vector<Eigen::Matrix3d> *Rs,
                     std::vector<Eigen::Vector3d> *Ts);

void SavePoints(std::string save_path, const std::vector<Eigen::Vector3d> &points);
void SavePointCloudAsPly(std::vector<Eigen::Vector3d> &points, std::string file_path);
void SaveGrayScaleImageFromPtr(double *pt, int width, int height, std::string file_path = "");
void Srand();
double Random();
double RandomLR(double l, double r);
}