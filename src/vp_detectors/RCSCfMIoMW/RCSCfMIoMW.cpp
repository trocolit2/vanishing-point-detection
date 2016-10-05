#include "RCSCfMIoMW.hpp"
#include "Tools.hpp"
#include <VanishingPointDetectionTools.hpp>

namespace vanishing_point{

RCSCfMIoMW::RCSCfMIoMW(uint ransac_iterations, double error_threshold){
  focus_ = 0;
  rotation_matrix_ = 0;
  ransac_iterations_ = ransac_iterations;
  error_threshold_ = error_threshold;
}

std::vector<cv::Point2f> RCSCfMIoMW::applyVPDetector(
                          cv::Mat image,
                          std::vector<cv::Vec4f> lines_segments,
                          std::vector<int> *line_id_by_vp){

  std::vector<cv::Point3f> lines(lines_segments.size());
  for (uint i = 0; i < lines_segments.size(); i++) {
    cv::Point2f initial_point(lines_segments[i][0], lines_segments[i][1]);
    cv::Point2f end_point(lines_segments[i][2], lines_segments[i][3]);
    lines[i] = defineEuclidianLineBy2Points(initial_point, end_point);
  }

  std::vector<int> line_cluster;
  std::vector<cv::Point2f> vps = RANSAC(lines_segments, lines,
     ransac_iterations_, line_cluster, error_threshold_);

  return vps;
}

double RCSCfMIoMW::getFocus(){
  return 0;
}

cv::Mat1f RCSCfMIoMW::getRotationMatrix(){
  return cv::Mat1f();
};


}
