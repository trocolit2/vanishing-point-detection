#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point{

cv::Mat drawOrthogonalVP(cv::Mat image, std::vector<cv::Point2f> points,
  bool draw_line = true);

}
