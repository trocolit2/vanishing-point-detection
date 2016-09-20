#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VPDetector_RCSCfMIoMW_Tools_test"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vp_detectors/RCSCfMIoMW/Tools.hpp>

using namespace vanishing_point;

BOOST_AUTO_TEST_CASE(centerPoint_testCase){

  std::vector<cv::Vec4f> line_segments = {cv::Vec4f(0,0,10,10),
    cv::Vec4f(-2,-2,10,10), cv::Vec4f(5,5,10,10), cv::Vec4f(5,5,-1,-1)};

  std::vector<cv::Point2f> gt_centers = {cv::Point2f(5,5), cv::Point2f(4,4),
    cv::Point2f(7.5,7.5), cv::Point2f(2,2)};

  for (uint i = 0; i < line_segments.size(); i++) {
    cv::Point2f center = lineSegmentCenterPoint(line_segments[i]);
    BOOST_CHECK_CLOSE(center.x, gt_centers[i].x, 0.01);
    BOOST_CHECK_CLOSE(center.y, gt_centers[i].y, 0.01);
    // std::cout << i << " Center "<< center << std::endl;
  }

}

BOOST_AUTO_TEST_CASE(distancePoint2Line_testCase){

  std::vector<cv::Point3f> lines = {cv::Point3f(2.0, -3, -4.0),
      cv::Point3f(6.0, -5.0, 10.0), cv::Point3f(3.0, 4.0, 0),
      cv::Point3f(3.0, -4.0, -25.0)};

  std::vector<cv::Point2f> points = {cv::Point2f(5,6), cv::Point2f(-3,7),
    cv::Point2f(2,-1), cv::Point2f(0,0)};
  std::vector<double> gt_distance = {3.328, 5.506, 0.400, 5.0};

  for (uint i = 0; i < lines.size(); i++) {
    double distance = distancePoint2Line(lines[i],points[i]);
    BOOST_CHECK_CLOSE(distance, gt_distance[i], 0.01);
    // Debug
    //std::cout << i << " distance "<< distance << std::endl;
  }
}

BOOST_AUTO_TEST_CASE(errorLineSegmentPoint2VP_testCase){

  std::vector<double> gt_error = {10, 5, 6, 100, 3, 9};

  cv::Point2f center(250, 250);
  std::vector<cv::Vec4f> line_segments = {
    cv::Vec4f(center.x+10,center.y, center.x-10,center.y),
    cv::Vec4f(center.x+5,center.y+5, center.x-5,center.y-5),
    cv::Vec4f(center.x+6,center.y+100, center.x-6,center.y-100),
    cv::Vec4f(center.x,center.y+100, center.x,center.y-100),
    cv::Vec4f(center.x+6,center.y+3, center.x-6,center.y-3),
    cv::Vec4f(center.x+20,center.y+9, center.x-20,center.y-9)};

  std::vector<cv::Point3f> homogeneos_vp = {
    cv::Point3f(250*0.5, 2*0.5, 0.4999), cv::Point3f(250*0.5, 2*0.5, 0.49999),
    cv::Point3f(250*0.5, 2*0.5, 0.499999), cv::Point3f(2*0.3, 250*0.3, 0.299),
    cv::Point3f(2*0.3, 250*0.3, 0.299), cv::Point3f(2*0.3, 250*0.3, 0.299)};
  for (uint i = 0; i < line_segments.size(); i++) {
    double error = errorLineSegmentPoint2VP(line_segments[i], homogeneos_vp[i]);
    BOOST_CHECK_CLOSE(error, gt_error[i], 0.01);
    // std::cout<<" I "<<i<< " error "<< error <<std::endl;
  }
}


BOOST_AUTO_TEST_CASE(estimationVPby4LinesCase1_testCase){

  cv::Mat3b image = cv::Mat3b::zeros(500,500);
  cv::Point2f center_point(image.cols/2, image.rows/2);
  std::vector<cv::Vec4f> line_segments = {
              cv::Vec4f(0.6, 0.1, 0.7, 0.1), cv::Vec4f(0.55, 0.2, 0.55, 0.3),
              cv::Vec4f(0.7, 0.4, 0.8, 0.5), cv::Vec4f(0.8, 0.7, 0.7, 0.8)};

  for (uint i = 0; i < line_segments.size(); i++) {
    cv::Mat temp_mat = cv::Mat(line_segments).row(i);
    for (uint j = 0; j < 2; j++) {
      cv::Mat1f local_mat(temp_mat);
      local_mat[0][j*2] = local_mat[0][j*2]*image.cols - center_point.x;
      local_mat[0][j*2+1] = local_mat[0][j*2+1]*image.rows - center_point.y;
    }
  }

  std::cout << "MAT VEC4F "<< cv::Mat(line_segments) << std::endl;

  std::vector<cv::Point2f> vps = estimationVPby4LinesCase1(line_segments);

  // std::cout << "VPS "<< cv::Mat(vps) << std::endl;

  for (uint i = 0; i < line_segments.size(); i++) {
    cv::Point2f point1(line_segments[i][0],line_segments[i][1]);
    cv::Point2f point2(line_segments[i][2],line_segments[i][3]);
    cv::line( image, point1 + center_point, point2 + center_point,
              cv::Scalar(0, 255, 255), image.rows * 0.004);
  }

  cv::imshow("out", image);
  cv::waitKey();

  // cv::Mat1f(line_segments).col(0) =(cv::Mat1f(line_segments).col(0)*image.cols);
  // cv::Mat1f(line_segments).col(2) = cv::Mat1f(line_segments).col(2)*image.cols;

}
