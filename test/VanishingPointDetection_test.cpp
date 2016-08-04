#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VanishingPointDetection_test"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <VanishingPointDetection.hpp>
#include <VanishingPointDetectionTools.hpp>

#define YUD_PATH "../../../resource/datasets/yorkurban/"
#define EURASIAN_PATH "../../../resource/datasets/eurasiancities/"

using namespace vanishing_point;

template <typename T> std::string numberToString(T number) {
  std::ostringstream ss;
  ss << number;
  return ss.str();
}

template <typename T>
std::string numberToString(T number, unsigned int number_zeros) {
  std::string raw_number = numberToString(number);
  unsigned int extra_zeros = number_zeros - raw_number.length();
  for (unsigned int i = 0; i < extra_zeros; i++) {
    raw_number = "0" + raw_number;
  }
  return raw_number;
}

BOOST_AUTO_TEST_CASE(standardVanishingPoint_testeCase) {

  VanishingPointDetection *detector = new VanishingPointDetection();
  std::vector<cv::Point2f> out = detector->applyDetector(cv::Mat());

  BOOST_CHECK_EQUAL(0, out.size());
}

BOOST_AUTO_TEST_CASE(checkEuclidianLineBy2Points_testcase) {

  std::vector<cv::Point2f> points_initial{cv::Point2f(2, 3), cv::Point2f(2, 3),
                                          cv::Point2f(2, 3)};

  std::vector<cv::Point2f> points_final{cv::Point2f(5, 7), cv::Point2f(8, 6),
                                        cv::Point2f(6, 4)};

  std::vector<cv::Point3f> lines{cv::Point3f(1.33, -1, 0.33333),
                                 cv::Point3f(0.5, -1, 2),
                                 cv::Point3f(1.0 / 4, -1, 5.0 / 2)};

  for (int i = 0; i < points_initial.size(); ++i) {
    cv::Point3f out =
        defineEuclidianLineBy2Points(points_initial[i], points_final[i]);
    BOOST_CHECK_CLOSE(lines[i].x, out.x, 1.0);
    BOOST_CHECK_CLOSE(lines[i].y, out.y, 1.0);
    BOOST_CHECK_CLOSE(lines[i].z, out.z, 1.0);
  }
}

BOOST_AUTO_TEST_CASE(checkEuclidianLineIntersection_testcase) {

  std::vector<cv::Point3f> lines_initial{
      cv::Point3f(2, 1, -4), cv::Point3f(1, 4, -7), cv::Point3f(2, 1, -3),
      cv::Point3f(0.44, -1, 3.4), cv::Point3f(2.25, -1, -85.2)};

  std::vector<cv::Point3f> lines_final{
      cv::Point3f(1, -1, 1), cv::Point3f(3, 1, 1), cv::Point3f(6, 5, 1),
      cv::Point3f(0.44, -1, -7.8), cv::Point3f(2.25, -1, -85.2)};

  std::vector<cv::Point2f> intersection_points{
      cv::Point2f(1, 2), cv::Point2f(-1, 2), cv::Point2f(4, -5),
      cv::Point2f(nanf(""), nanf("")), cv::Point2f(26.5, -25.5)};

  for (int i = 0; i < lines_initial.size(); ++i) {
    cv::Point2f final_point = definePointByEuclidianLinesIntersection(
        lines_initial[i], lines_final[i]);

    if (std::isnan(intersection_points[i].x)) {
      BOOST_CHECK_EQUAL(std::isnan(intersection_points[i].x),
                        std::isnan(final_point.x));
      BOOST_CHECK_EQUAL(std::isnan(intersection_points[i].y),
                        std::isnan(final_point.y));
    } else {
      BOOST_CHECK_CLOSE(intersection_points[i].x, final_point.x, 1.0);
      BOOST_CHECK_CLOSE(intersection_points[i].y, final_point.y, 1.0);
    }
  }
}

BOOST_AUTO_TEST_CASE(drawOrthogonalVP_testeCase) {

  cv::Mat3b temp_image = cv::Mat3b::zeros(500, 500);
  std::vector<cv::Point2f> points;
  points.push_back(cv::Point2f(400, 230)); // VP X -> RED
  points.push_back(cv::Point2f(100, 200)); // VP Y -> GREEN
  points.push_back(cv::Point2f(250, 400)); // VP Z -> BLUE

  cv::Point2f principal_point(temp_image.cols / 2, temp_image.rows / 2);
  temp_image = drawOrthogonalVP(temp_image, points, principal_point);
  std::vector<cv::Mat> channels;
  cv::split(temp_image, channels);
  for (unsigned int i = 0; i < points.size(); i++) {
    cv::Point max_point;
    cv::minMaxLoc(channels[2 - i], 0, 0, 0, &max_point);
    if (i % 2)
      BOOST_CHECK_CLOSE(points[i].y, max_point.y, 3.0);
    else
      BOOST_CHECK_CLOSE(points[i].x, max_point.x, 3.0);
  }
}

BOOST_AUTO_TEST_CASE(checkYUDDataset_testCase) {

  std::string gt_path = std::string(YUD_PATH) + "gt_data.yml";

  cv::Mat raw_horizon_lines_gt;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs["horizon_lines"] >> raw_horizon_lines_gt;
  // std::cout << " out vpd " << raw_horizon_lines_gt << std::endl;

  cv::Mat1f horizon_lines_gt(raw_horizon_lines_gt);

  std::vector<cv::Point2f> points(3);
  std::vector<cv::Point3f> points_3d(3);

  for (unsigned int k = 0; k < horizon_lines_gt.rows; k++) {
    std::string path_image =
        std::string(YUD_PATH) + numberToString(k + 1, 3) + ".jpg";

    // std::cout << " path_image " << path_image << std::endl;
    cv::Mat yud_image = cv::imread(path_image);
    cv::Point3f horizon_line(horizon_lines_gt[k][0], horizon_lines_gt[k][1],
                             horizon_lines_gt[k][2]);

    yud_image = drawHorizonLine(yud_image, horizon_line);
    // cv::imshow("horizon line York Urban", yud_image);
    // cv::waitKey();
  }
}

BOOST_AUTO_TEST_CASE(checkEurasianDataset_testCase) {

  std::string gt_path = std::string(EURASIAN_PATH) + "gt_data.yml";

  cv::Mat raw_horizon_lines_gt, raw_zenith_gt;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs["horizon_lines"] >> raw_horizon_lines_gt;
  fs["zenith"] >> raw_zenith_gt;
  //  std::cout << " out vpd " << raw_horizon_lines_gt << std::endl;

  cv::Mat1f horizon_lines_gt(raw_horizon_lines_gt);
  cv::Mat1f zenith_gt(raw_zenith_gt);

  std::vector<cv::Point2f> points(3);
  std::vector<cv::Point3f> points_3d(3);

  for (unsigned int k = 0; k < horizon_lines_gt.rows; k++) {
    std::string path_image =
        std::string(EURASIAN_PATH) + numberToString(k + 1, 3) + ".jpg";

    // std::cout << " path_image " << path_image << std::endl;
    cv::Mat image = cv::imread(path_image);
    cv::Point3f horizon_line(horizon_lines_gt[k][0], horizon_lines_gt[k][1],
                             horizon_lines_gt[k][2]);
    cv::Point2f zenith(zenith_gt[k][0], zenith_gt[k][1]);
    cv::Point2f principal_point(image.cols / 2, image.rows / 2);

    cv::Point3f zenith_line =
        defineEuclidianLineBy2Points(zenith, principal_point);
    cv::Point2f intersection_point =
        definePointByEuclidianLinesIntersection(horizon_line, zenith_line);

    image = drawHorizonLine(image, horizon_line);
    image = drawZenithLine(image, zenith, principal_point, intersection_point);
    cv::imshow("horizon line Euroasian Cites", image);
    cv::waitKey();
  }
}
