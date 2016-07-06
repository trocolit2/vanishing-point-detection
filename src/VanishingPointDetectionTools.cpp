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

cv::Mat drawHorizonLine(cv::Mat image, cv::Point3f line) {

  cv::Point2f initial_point(0, 0), final_point(image.cols, 0);
  initial_point.y = (line.x * initial_point.x + line.z) / -line.y;
  final_point.y = (line.x * final_point.x + line.z) / -line.y;

  std::cout << " line " << line << std::endl;
  std::cout << " initial " << initial_point;
  std::cout << " final   " << final_point << std::endl;

  cv::Scalar color(255, 0, 255);
  cv::line(image, initial_point, final_point, color, image.rows * 0.008);
  return image;
}

cv::Mat drawZenithLine(cv::Mat image, cv::Point2f zenith_point,
                       cv::Point2f central_point) {

  return image;
}
}
