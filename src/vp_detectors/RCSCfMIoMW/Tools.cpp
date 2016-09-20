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

cv::Point2f definePointByEuclidianLinesIntersection(cv::Point3f line_initial,
                                                    cv::Point3f line_final) {

  cv::Mat1f mat_lines = cv::Mat1f::ones(2, 3);
  mat_lines.row(0) = cv::Mat1f(line_initial).t();
  mat_lines.row(1) = cv::Mat1f(line_final).t();
  // std::cout << "Mat with lines" << mat_lines << std::endl;

  cv::Mat1f temp_point;
  cv::SVD::solveZ(mat_lines, temp_point);
  // std::cout << "Point raw" << temp_point.t() << std::endl;

  cv::Point2f intersection_point(nanf("There is no intersection"),
                                 nanf("There is no intersection"));
  float epslon = 1e-8;
  if (fabs(temp_point[2][0]) > epslon) {
    intersection_point.x = temp_point[0][0] / temp_point[2][0];
    intersection_point.y = temp_point[1][0] / temp_point[2][0];
  }

  return intersection_point;
}

std::vector<cv::Point2f> estimationVPby4LinesCase1(
                                          std::vector<cv::Vec4f> line_segments,
                                          double *focal_length){
  std::vector<cv::Point3f> lines(4);
  for (uint i = 0; i < line_segments.size(); i++) {
    cv::Point2f initial_point(line_segments[i][0], line_segments[i][1]);
    cv::Point2f end_point(line_segments[i][2], line_segments[i][3]);
      lines[i] = defineEuclidianLineBy2Points(initial_point, end_point);
  }

  std::vector<cv::Point2f> vps(3);
  for (uint j = 0; j < 2; j++)
    vps[j] = definePointByEuclidianLinesIntersection( lines[j*2], lines[j*2-1]);

  double local_focal = -(vps[0].x*vps[1].x + vps[0].y*vps[1].y);
  local_focal = sqrt(local_focal);

  cv::Mat1f mat_K = cv::Mat1f::zeros(3,3);
  mat_K[0][0] = local_focal;
  mat_K[1][1] = local_focal;
  mat_K[2][2] = 1;
  cv::Mat1f mat_K_inv = mat_K.inv(); // can optimeze

  cv::Point3f vp0(vps[0].x,vps[0].y,1);
  cv::Point3f vp1(vps[1].x,vps[1].y,1);

  cv::Mat1f vp2 = mat_K_inv * cv::Mat1f(vp0); // can optimeze
  vp2 = vp2.cross(mat_K_inv * cv::Mat1f(vp1));
  vp2 = mat_K * vp2;
  vps[2] = cv::Point2f(vp2[0][0]/vp2[0][2], vp2[0][1]/vp2[0][2]);

  if(focal_length)
    (*focal_length) = local_focal;
    
  return vps;
}



}
