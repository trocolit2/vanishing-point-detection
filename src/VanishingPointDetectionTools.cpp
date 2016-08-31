#include <VanishingPointDetectionTools.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
namespace vanishing_point {

cv::Mat drawOrthogonalVP(cv::Mat image, std::vector<cv::Point2f> points,
                         cv::Point2f principal_point) {

  for (unsigned int i = 0; i < points.size(); i++) {
    cv::Scalar color(255 * ((i % 3) == 2), 255 * ((i % 3) == 1),
                     255 * ((i % 3) == 0));
    cv::circle(image, points[i], image.rows * 0.01, color, -1);

    if (principal_point.x != -1 && principal_point.y != -1)
      cv::line(image, principal_point, points[i], color, image.rows * 0.008);
  }
  return image;
}

cv::Mat drawHorizonLine(cv::Mat original_image, cv::Point3f line) {

  cv::Mat image;
  original_image.copyTo(image);
  cv::Point2f initial_point(0, 0), final_point(image.cols, 0);
  initial_point.y = (line.x * initial_point.x + line.z) / -line.y;
  final_point.y = (line.x * final_point.x + line.z) / -line.y;

  cv::Scalar color(255, 0, 255);
  cv::line(image, initial_point, final_point, color, image.rows * 0.004);
  return image;
}

cv::Mat drawZenithLine( cv::Mat original_image,
                        cv::Point2f zenith_point,
                        cv::Point2f central_point,
                        cv::Point2f intersection_point) {

  cv::Mat image;
  original_image.copyTo(image);
  cv::Scalar black(0, 0, 0);
  cv::Scalar yellow(0, 255, 255);
  cv::Scalar purple(255, 0, 255);

  cv::Point2f end_point;

  if( central_point.y > zenith_point.y ){
    central_point.y < intersection_point.y ? end_point = intersection_point
                                           : end_point = central_point;
  }else{
    central_point.y < intersection_point.y ? end_point = central_point
                                           : end_point = intersection_point;
  }


  cv::line(image, zenith_point, end_point, yellow, image.rows * 0.004);

  // cv::circle(image, zenith_point, image.rows * 0.008, black, 3);
  cv::circle(image, zenith_point, image.rows * 0.004, yellow, -1);

  cv::circle(image, central_point, image.rows * 0.008, black, 3);
  cv::circle(image, central_point, image.rows * 0.008, yellow, -1);

  cv::circle(image, intersection_point, image.rows * 0.008, black, 3);
  cv::circle(image, intersection_point, image.rows * 0.008, purple, -1);

  return image;
}

cv::Point3f adjustPointsToDraw( cv::Point2f zenith_point,
                                cv::Point2f principal_point,
                                cv::Point3f horizon_line,
                                cv::Point2f *intersection_point,
                                cv::Point2f *zenith_local){
  cv::Point3f zenith_line =
    defineEuclidianLineBy2Points(zenith_point, principal_point);

  (*intersection_point) =
    definePointByEuclidianLinesIntersection(horizon_line, zenith_line);


  if( zenith_point.y < 0 )
    if(zenith_line.y == 0)
      (*zenith_local) = cv::Point2f(principal_point.x, 0);
    else
      (*zenith_local) = cv::Point2f(-zenith_line.z / zenith_line.x, 0);

  else if( zenith_point.y > (principal_point.y * 2) )
    if(zenith_line.y == 0)
      (*zenith_local) = cv::Point2f(principal_point.x,
                                    principal_point.y * 2);
    else
      (*zenith_local) =
        cv::Point2f (((principal_point.y * 2) - zenith_line.z )/ zenith_line.x,
                    (principal_point.y * 2));
  else
    (*zenith_local) = zenith_point;

return zenith_line;
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

cv::Point3f computeHorizonLineByZenithPointAndLineSamples(
                                      cv::Point3f zenith_line,
                                      std::vector<cv::Point3f> line_samples){
  return cv::Point3f();

}

cv::Point3f horizonLineEstimation(cv::Point2f zenith,
                                  cv::Point2f principal_point,
                                  std::vector<cv::Point2f> vanishing_points,
                                  std::vector<int> lines_by_vp,
                                  cv::Point3f *horizon_without_zenith){

  // define horizon_line slope by zenith_line slope.
  cv::Point3f zenith_line =
                  defineEuclidianLineBy2Points(zenith, principal_point);

  // zenith_line slope is horizon_line perpendicular
  float slope_horizon = fabs(zenith_line.x);

  slope_horizon == 1.0 ?  slope_horizon = 0 :
                            slope_horizon = -1.0/zenith_line.x;

  // count sample number
  int total_lines_samples = 0;
  for (int k = 0; k < lines_by_vp.size(); k++)
    total_lines_samples+=lines_by_vp[k];

  cv::Mat1f left_mat = cv::Mat1f::ones(total_lines_samples,1);
  cv::Mat1f right_mat = cv::Mat1f::zeros(total_lines_samples,1);

  //fill matrix with sample values
  int count = 0;
  for (int i = 0; i < lines_by_vp.size(); i++) {
    for (int j = 0; j < lines_by_vp[i]; j++)
      right_mat[count + j][0] = -slope_horizon * vanishing_points[i].x +
                                vanishing_points[i].y;
    count+= lines_by_vp[i];
  }

  // solve 1D least square to find horizon Y intercept value
  cv::Mat1f out;
  cv::solve(left_mat, right_mat, out, cv::DECOMP_NORMAL);
  cv::Point3f horizon_line(slope_horizon, -1, out[0][0]);

  return horizon_line;
}







}
