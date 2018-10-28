#include <line3D.h>
// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// std
#include <cstdio>
#include <string>

void AddImgs(std::string file_path, L3DPP::Line3D *line_3D) {
  int num_cam = 32;
  
  
  auto matrix_file_name = file_path + "matrix.txt"
  FILE *f = fopen(file_path.c_str(), "r");
  for (int i = 0; i < num_cam; i++) {
    // K, R, t
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        float val;
        fscanf(f, "%f", &val);
        
      }
  }
  fclose(f)

  for (int i = 0; i < num_cam; i++) {
    auto file_name = file_path + std::string("") + std::to_string(num_cam) + std::string(".png");
    cv::Mat img = cv::imread(file_name);
    
    line_3D -> addImage(i, img)
  }
}

int main() {
  auto line_3D = new L3DPP::Line3D(
    std::string(""),
    L3D_DEF_LOAD_AND_STORE_SEGMENTS,
    L3D_DEF_MAX_IMG_WIDTH,
    L3D_DEF_MAX_NUM_SEGMENTS,
    true,
    false
  );
  AddImgs(std::string("./"), line_3D);
  return 0;
}