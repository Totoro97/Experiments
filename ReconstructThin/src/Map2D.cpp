#include "Map2D.h"

// std
#include <pair>
#include <queue>

Map2D::Map2D(const cv::mat& img) {
  height_ = img.rows;
  width_ = img.cols;
  map2d_ = new int[height_ * width_];
  std::fill_n(map2d_, height_ * width_, -1);
  ed::Ed ed;
  auto edges = ed.detectEdges(img);

  std::queue<std::pair<int, int>> que;
  for (const auto &edge : edges) {
    for (const auto &pix : edge) {
      map2d_[pix.y * width_ + pix.x] = 0;
      que.push(std::make_pair(pix.y, pix.x));
    }
  }

  // BFS
  const int x_bias[4] = { -1, 1, 0, 0 }
  
  while (!que.empty()) {
    auto pix = que.front();
    que.pop();
    
  }
}

Map2D::~Map2D() {
  delete(map2d_);
}