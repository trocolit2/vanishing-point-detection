#include "Tools.hpp"
#include <VanishingPointDetectionTools.hpp>

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

std::vector<cv::Point2f> estimationVPby4LinesCase1(
                                          std::vector<cv::Point3f> lines,
                                          double *focal_length){

  std::vector<cv::Point2f> vps(3);
  for (uint j = 0; j < 2; j++)
    vps[j] = definePointByEuclidianLinesIntersection( lines[j*2], lines[j*2+1]);

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

std::vector<cv::Point2f> estimationVPby4LinesCase2(
                                          std::vector<cv::Point3f> lines,
                                          double *focal_length){
  std::vector<cv::Point2f> vps(3);
  vps[0] = definePointByEuclidianLinesIntersection(lines[0], lines[1]);

  cv::Point3f consts(0,0,0);
  consts.x =                        lines[2].x*lines[3].x
             +                      lines[2].y*lines[3].y;

  consts.y =   vps[0].y*  vps[0].y* lines[2].x*lines[3].x
             - vps[0].x*  vps[0].y* lines[2].y*lines[3].x
             - vps[0].x*            lines[2].z*lines[3].x
             - vps[0].x*  vps[0].y* lines[2].x*lines[3].y
             - vps[0].y*            lines[2].z*lines[3].y
             - vps[0].x*            lines[2].x*lines[3].z
             - vps[0].y*            lines[2].y*lines[3].z
             - vps[0].x*  vps[0].x* lines[2].y*lines[3].y;

  consts.z =   vps[0].x*            lines[2].z*lines[3].z
             + vps[0].y*  vps[0].y* lines[2].z*lines[3].z;

  double delta = (consts.y*consts.y)-4*consts.x*consts.z;

  if(delta < 0){
    if(focal_length)
      (*focal_length) = -1;
    return std::vector<cv::Point2f>(3);
  }

  delta = sqrt(delta);
  double focal_length_2 = ((+consts.y)+delta)/2*consts.x;

  cv::Mat1f mat_h = cv::Mat1f::ones(1,3);
  mat_h[0][0] = vps[0].x/focal_length_2;
  mat_h[0][1] = vps[0].y/focal_length_2;

  cv::Mat1f mat_vp1 = mat_h.cross(cv::Mat(lines[2]));
  cv::Mat1f mat_vp2 = mat_h.cross(cv::Mat(lines[3]));

  vps[1] = cv::Point2f( mat_vp1[0][0]/mat_vp1[0][2],
                        mat_vp1[0][1]/mat_vp1[0][2]);

  vps[2] = cv::Point2f( mat_vp2[0][0]/mat_vp1[0][2],
                        mat_vp2[0][1]/mat_vp1[0][2]);

  if(focal_length)
    (*focal_length) = sqrt(focal_length_2);

  return vps;
}

}
