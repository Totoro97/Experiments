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

  Map2D(int height, int width, const cv::mat &img);
};