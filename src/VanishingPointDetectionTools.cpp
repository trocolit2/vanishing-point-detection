#include <VanishingPointDetectionTools.hpp>
#include <iostream>
namespace vanishing_point {

cv::Mat drawOrthogonalVP(cv::Mat image, std::vector<cv::Point2f> points,
                         cv::Point2f principal_point) {

  for (unsigned int i = 0; i < points.size(); i++) {
    cv::Scalar color(255 * ((i % 3) == 2), 255 * ((i % 3) == 1),
                     255 * ((i % 3) == 0));
    cv::circle(image, points[i], image.rows * 0.01, color, -1);

    if (principal_point.x != 0 && principal_point.y != 0)
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

  // std::cout << " line " << line << std::endl;
  // std::cout << " initial " << initial_point;
  // std::cout << " final   " << final_point << std::endl;

  cv::Scalar color(255, 0, 255);
  cv::line(image, initial_point, final_point, color, image.rows * 0.004);
  return image;
}

cv::Mat drawZenithLine(cv::Mat original_image, cv::Point2f zenith_point,
                       cv::Point2f central_point) {

  cv::Mat image;
  original_image.copyTo(image);
  cv::circle(image, zenith_point, image.rows * 0.01, cv::Scalar(0, 0, 0), 3);
  cv::circle(image, zenith_point, image.rows * 0.01, cv::Scalar(0, 255, 255),
             -1);
  cv::circle(image, central_point, image.rows * 0.01, cv::Scalar(0, 0, 0), 3);
  cv::circle(image, central_point, image.rows * 0.01, cv::Scalar(0, 255, 255),
             -1);

  return image;
}

cv::Point3f defineEuclidianLineBy2Points(cv::Point2f point_inital,
                                         cv::Point2f point_final) {
  return cv::Point3f(0, 0, 0);
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
}
