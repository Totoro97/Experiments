#include "Headers.h"
#include "DistriMap.h"
// std
#include <vector>

struct Tube {
  Eigen::Vector2d a, b;
  double r;
};

namespace Algo {

//void SuitWithTube(const cv::Mat &img, std::vector<Tube> &tubes, int num_tube);

void SuitWithPointCloud(std::vector<DistriMap *> &distri_maps,
                        std::vector<Eigen::Vector3d> &points,
                        int num_point,
                        int num_ter = 100,
                        double init_range = 1.0);
}
