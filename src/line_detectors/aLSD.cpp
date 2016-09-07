#include <aLSD.hpp>

extern "C" {
  #include <aLSD/lsd.h>
}

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace vanishing_point{

aLSD::aLSD(){
  _time_values = std::vector<double>();
  _time_sessions = std::vector<std::string>();
}

std::vector<cv::Vec4f> aLSD::applyLSDetector(cv::Mat image){

  // convert BGR color to gray color
  cv::Mat temp;
  cv::cvtColor(image, temp, CV_BGR2GRAY);

  double *img_pointer;
  int X = image.cols; /* x image size */
  int Y = image.rows; /* y image size */

  /* create a simple image: left half black, right half gray */
  img_pointer = (double *) malloc(X * Y * sizeof(double));
  if (img_pointer == NULL) {
    fprintf(stderr, "error: not enough memory\n");
    exit(EXIT_FAILURE);
  }

  int x, y;
  for (x = 0; x < X; x++)
    for (y = 0; y < Y; y++)
      img_pointer[x + y * X] = temp.at<uint8_t>(y, x); /* image(x,y) */

  /* LSD call */
  int n;
  double *out;
  out = lsd(&n, img_pointer, X, Y);

  std::vector<cv::Vec4f> line_segments;
  for (int i = 0; i < n; i++) {
    cv::Vec4f line_segment(out[7 * i + 0], out[7 * i + 1], // point1
                            out[7 * i + 2], out[7 * i + 3]); // point2
    // cv::Point2f point1(out[7 * i + 0], out[7 * i + 1]), point2(out[7 * i + 2],
    //                                                            out[7 * i + 3]);
    // Line2D line(point1, point2);
    line_segments.push_back(line_segment);
  }

  /* free memory */
  free((void *) img_pointer);
  free((void *) out);

  return line_segments;
}

}
