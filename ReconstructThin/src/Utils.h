// Eigen
#include <eigen3/Eigen/Eigen>
// std
#include <string>
#include <vector>

namespace Utils {

void ReadKRTFromFile(std::string file_path, int cam_num,
                     std::vector<Eigen::Matrix3d> *Ks,
                     std::vector<Eigen::Matrix3d> *Rs,
                     std::vector<Eigen::Vector3d> *Ts);
}