#include <VanishingPointDetectionTools.hpp>

namespace vanishing_point{

cv::Mat drawOrthogonalVP(cv::Mat image, std::vector<cv::Point2f> points,
  bool draw_line){

  for (unsigned int i = 0; i < points.size(); i++) {
    cv::Scalar color(255*((i%3)==2), 255*((i%3)==1), 255*((i%3)==0));
    cv::circle(image, points[i], image.rows*0.01,color,-1);

    if(draw_line)
      cv::line(image, cv::Point2f(image.cols/2,image.rows/2),
        points[i], color, image.rows*0.008);
  }
  return image;
}


}
