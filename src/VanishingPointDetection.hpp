#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point{

class VanishingPointDetection{

public:
  VanishingPointDetection(){};

  std::vector<cv::Point2f> applyVPDetector(cv::Mat image){
    return std::vector<cv::Point2f>();
  };

};
}
