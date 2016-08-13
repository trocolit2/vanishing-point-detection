#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point {

cv::Mat drawOrthogonalVP(cv::Mat image, std::vector<cv::Point2f> points,
                         cv::Point2f principal_point = cv::Point2f(-1, -1));

cv::Mat drawHorizonLine(cv::Mat image, cv::Point3f line);

cv::Mat drawZenithLine(cv::Mat original_image, cv::Point2f zenith_point,
                       cv::Point2f central_point,
                       cv::Point2f intersection_point);

cv::Point3f adjustPointsToDraw( cv::Point2f zenith_point,
                                cv::Point2f principal_point,
                                cv::Point3f horizon_line,
                                cv::Point2f *intersection_point,
                                cv::Point2f *zenith_local);


cv::Point3f defineEuclidianLineBy2Points(cv::Point2f point_inital,
                                         cv::Point2f point_final);

cv::Point2f definePointByEuclidianLinesIntersection(cv::Point3f line_initial,
                                                    cv::Point3f line_final);
}
