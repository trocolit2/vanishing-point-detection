#include <Tools.hpp>
#include <iostream>

namespace vanishing_point {

cv::Point2f lineSegmentCenterPoint(cv::Vec4f line_segment){
  double center_x = (line_segment[0]+line_segment[2])/2;
  double center_y = (line_segment[1]+line_segment[3])/2;
  return cv::Point2f(center_x, center_y);
};

double distancePoint2Line(cv::Point3f line, cv::Point2f point){
  double distance = line.x*point.x + line.y*point.y + line.z;
  distance = abs(distance)/(sqrt(line.x*line.x + line.y*line.y));
  return distance;
}

double errorLineSegmentPoint2VP(  cv::Vec4f line_segment,
                                  cv::Point3f homogeneo_vp,
                                  cv::Point2f *center_point,
                                  cv::Point2f *end_point,
                                  cv::Point3f *line_center_vp){

  cv::Point2f local_center_point = lineSegmentCenterPoint(line_segment);
  cv::Point2f vp( homogeneo_vp.x/homogeneo_vp.z,
                  homogeneo_vp.y/homogeneo_vp.z);

  cv::Point2f local_end_point(line_segment[0],line_segment[1]);
  cv::Point3f line = defineEuclidianLineBy2Points(local_center_point, vp);
  if(center_point && end_point && line_center_vp){

  }


  return distancePoint2Line(line, local_end_point);
}

cv::Point3f defineEuclidianLineBy2Points(cv::Point2f point_inital,
                                         cv::Point2f point_final) {

  float delta_X = point_final.x - point_inital.x;
  float slope,  intercept;
  cv::Point3f line;

  if( delta_X != 0 ){
    slope = (point_final.y - point_inital.y) / delta_X;
    intercept = point_inital.y - slope * point_inital.x;
    line = cv::Point3f(slope, -1, intercept);
  }else{
    line = cv::Point3f(-1, 0, point_final.x);
  }

  return line;
}


}
