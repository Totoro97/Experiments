// edge draw
#include <ed.hpp>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

// std

class Map2D {
public:
  int height_, width_;
  int *map2d_;
  Eigen::Matrix3d K_, R_;
  Eigen::Vector3d T_;

  Map2D(int height, int width, const cv::mat &img);

  double MinDist2Edge(Eigen::Vector3d pt) const;
};