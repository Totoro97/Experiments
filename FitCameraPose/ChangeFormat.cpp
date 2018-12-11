//
// Created by Aska on 2018/12/11.
//

#include "Headers.h"
#include <string>

int main() {
  for (int i = 0; i < 64; i++) {
    auto img = cv::imread(std::string("C:/Users/Aska/tmp/") + std::to_string(i) + std::string(".png"));
    cv::imwrite(std::string("C:/Users/Aska/tmp/jpg/") + std::to_string(i) + std::string(".jpg"), img);
  }
  return 0;
}