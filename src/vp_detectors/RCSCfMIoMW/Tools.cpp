#include <Tools.hpp>

namespace vanishing_point {

cv::Point2f lineSegmentCenterPoint(cv::Vec4f line_segment){
  double center_x = (line_segment[0]+line_segment[2])/2;
  double center_y = (line_segment[1]+line_segment[3])/2;
  return cv::Point2f(center_x, center_y);
};

double distancePoint2Line(cv::Point3f line, cv::Point2f point){
  return 0;
}

double errorLineSegmentPoint2VP(  cv::Vec4f line_segment,
                                  cv::Point3f homogeneos_vp){
  return 0;
}

}
