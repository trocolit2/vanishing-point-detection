#include <opencv2/core/core.hpp>

namespace vanishing_point {

cv::Point2f lineSegmentCenterPoint(cv::Vec4f line_segment);

double distancePoint2Line(cv::Point3f line, cv::Point2f point);

double errorLineSegmentPoint2VP(  cv::Vec4f line_segment,
                                  cv::Point3f homogeneos_vp,
                                  cv::Point2f *center_point = 0,
                                  cv::Point2f *end_point = 0,
                                  cv::Point3f *line_center_vp = 0);

// duplicated code from VanishingPointDetectionTools
cv::Point3f defineEuclidianLineBy2Points(cv::Point2f point_inital,
                                         cv::Point2f point_final);

// duplicated code from VanishingPointDetectionTools
cv::Point2f definePointByEuclidianLinesIntersection(cv::Point3f line_initial,
                                                   cv::Point3f line_final);


// to apply this method, principal point is setted as (0,0)
std::vector<cv::Point2f> estimationVPby4LinesCase1(
                                          std::vector<cv::Vec4f> line_segments,
                                          double *focal_length = 0);


// to apply this method, principal point is setted as (0,0)
std::vector<cv::Point2f> estimationVPby4LinesCase2(
                                          std::vector<cv::Vec4f> line_segments,
                                          double *focal_length = 0);

}
