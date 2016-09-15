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

}
