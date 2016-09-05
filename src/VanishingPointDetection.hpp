#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point{

class VanishingPointDetection{

public:

  std::vector<cv::Point2f> applyVPDetector(
                                cv::Mat image,
                                std::vector<int> *line_id_by_vp = 0,
                                std::vector<cv::Vec4f> *lines_segments = 0 ){

    return std::vector<cv::Point2f>();
  };

};
}
