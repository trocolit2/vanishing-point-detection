#include <opencv2/core/core.hpp>

namespace vanishing_point {

cv::Point2f lineSegmentCenterPoint(cv::Vec4f line_segment);

double distancePoint2Line(cv::Point3f line, cv::Point2f point);

double errorLineSegmentPoint2VP(  cv::Vec4f line_segment,
                                  cv::Point3f homogeneos_vp);

}
