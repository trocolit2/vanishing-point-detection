// Method based on LSD: a Line Segment Detector, 2010. Class name is the paper
// title acronym: aLSD.

#include "LineSegmentDetector.hpp"
#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point{

class aLSD : public LineSegmentDetector {

  public:
    aLSD();
    std::vector<cv::Vec4f> applyLSDetector(cv::Mat image);

};
}
