#include <aLSD.hpp>
extern "C" {
  #include <aLSD/lsd.h>
}

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <ctime>
#include <TimeCountTools.hpp>

clock_t begin_time;

namespace vanishing_point{

aLSD::aLSD(){
  _time_sessions = {"CV - CVT BGR2GRAY", "C - MALOC X*Y size",
                    "CV - Copy CV::Mat to C vector ", "C - Run LSD CORE ",
                    "CV - convert LSD to CV points "};
  _time_values = std::vector<double>(_time_sessions.size(), 0);
}

std::vector<cv::Vec4f> aLSD::applyLSDetector(cv::Mat image){

  // _time_sessions[0] =
  // begin_time = std::clock();
  cv::Mat temp;
  // convert BGR color to gray color
  TIME_COUNT(_time_values[0]){
  cv::cvtColor(image, temp, CV_BGR2GRAY);
  }

  double *img_pointer;
  int X = image.cols; /* x image size */
  int Y = image.rows; /* y image size */


  TIME_COUNT(_time_values[1]){
  /* create a simple image: left half black, right half gray */
  img_pointer = (double *) malloc(X * Y * sizeof(double));
  }
  if (img_pointer == NULL) {
    fprintf(stderr, "error: not enough memory\n");
    exit(EXIT_FAILURE);
  }

  int x, y;
  TIME_COUNT(_time_values[2]){
  for (x = 0; x < X; x++)
    for (y = 0; y < Y; y++)
      img_pointer[x + y * X] = temp.at<uint8_t>(y, x); /* image(x,y) */
  }
  /* LSD call */
  int n;
  double *out;
  TIME_COUNT(_time_values[3]){
  out = lsd(&n, img_pointer, X, Y);
  }


  std::vector<cv::Vec4f> line_segments;
  TIME_COUNT(_time_values[4]){
  for (int i = 0; i < n; i++) {
    cv::Vec4f line_segment(out[7 * i + 0], out[7 * i + 1], // point1
                            out[7 * i + 2], out[7 * i + 3]); // point2
    line_segments.push_back(line_segment);
  }
  }

  /* free memory */
  free((void *) img_pointer);
  free((void *) out);

  return line_segments;
}

}
