#include <RCSCfMIoMW.hpp>

namespace vanishing_point{

RCSCfMIoMW::RCSCfMIoMW(){

}


std::vector<cv::Point2f> RCSCfMIoMW::applyVPDetector(
                          cv::Mat image,
                          std::vector<int> *line_id_by_vp,
                          std::vector<cv::Vec4f> *lines_segments){

  return std::vector<cv::Point2f>();
}

}
