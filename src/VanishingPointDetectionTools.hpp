#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point {

cv::Mat drawOrthogonalVP(cv::Mat image, std::vector<cv::Point2f> points,
                         cv::Point2f principal_point = cv::Point2f(0, 0));

cv::Mat drawHorizonLine(cv::Mat image, cv::Point3f line);
cv::Mat drawZenithLine(cv::Mat image, cv::Point2f zenith_point,
                       cv::Point2f central_point = cv::Point2f(-1, -1));
}
