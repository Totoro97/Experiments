#include "GlobalCalibrator.h"
#include "Thinning.h"

GlobalCalibrator::GlobalCalibrator(std::string dir_name, int num_frame): num_frame_(num_frame) {
  octree_ = new Octree(Eigen::Vector3d(0.0, 0.0, 0.0), 2.0, 0.03125 / 2.0);
  for (int idx = 0; idx < num_frame; idx++) {
    auto img = cv::imread(dir_name + "/" + std::to_string(idx) + ".png");
    cv::Mat img_gray(img.rows, img.cols, CV_8UC1);
    for (int i = 0; i < img.rows * img.cols; i++) {
      if (img.data[i * 3] == img.data[0] &&
          img.data[i * 3 + 1] == img.data[1] &&
          img.data[i * 3 + 2] == img.data[2]) {
        img_gray.data[i] = 0;
      }
      else
        img_gray.data[i] = (uchar) 255;
    }

    thinning(img_gray, img_gray);
    calibrators_.push_back(new Calibrator(img_gray, octree_, idx, (double) idx * M_PI * 2.0 / 16.0));
    std::cout << "Pix size: " << calibrators_.back()->visible_pixs_.size() << std::endl;
  }
}

GlobalCalibrator::~GlobalCalibrator() {
  for (auto calibrator : calibrators_) {
    delete(calibrator);
  }
  delete(octree_);
}

// No depth parameters (Estimate from Octree)

void GlobalCalibrator::Run() {
  std::srand(173);
  calibrators_[0]->AddToOctree();
  for (auto iter = std::next(calibrators_.begin()); iter != calibrators_.end(); iter++) {
    auto calibrator = *iter;
    // calibrator->CopyCamParasFrom(*std::prev(iter));
    calibrator->Calibrate();
    calibrator->AddToOctree();
  }
  octree_->OutputMesh("tmp.ply", 0.03125 / 2.0, 0.999 * 8.0);
}