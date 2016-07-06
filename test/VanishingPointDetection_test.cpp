#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VanishingPointDetection_test"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <VanishingPointDetection.hpp>
#include <VanishingPointDetectionTools.hpp>

#define YUD_PATH "../../../resource/yud_dataset/"
#define EURASIAN_PATH "../../../resource/eurasiancitiesbase/"

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
    BOOST_CHECK_CLOSE(points[i].x, max_point.x, 3.0);
    BOOST_CHECK_CLOSE(points[i].y, max_point.y, 3.0);
  }
}

BOOST_AUTO_TEST_CASE(checkYUDDataset_testCase) {

  std::string gt_path = std::string(YUD_PATH) + "gt_data.yml";

  cv::Mat raw_horizon_lines_gt;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs["horizon_lines"] >> raw_horizon_lines_gt;
  std::cout << " out vpd " << raw_horizon_lines_gt << std::endl;

  cv::Mat1f horizon_lines_gt(raw_horizon_lines_gt);

  std::vector<cv::Point2f> points(3);
  std::vector<cv::Point3f> points_3d(3);

  for (unsigned int k = 0; k < horizon_lines_gt.rows; k++) {
    std::string path_image =
        std::string(YUD_PATH) + numberToString(k + 1, 3) + ".jpg";

    std::cout << " path_image " << path_image << std::endl;
    cv::Mat yud_image = cv::imread(path_image);
    cv::Point3f horizon_line(horizon_lines_gt[k][0], horizon_lines_gt[k][1],
                             horizon_lines_gt[k][2]);

    drawHorizonLine(yud_image, horizon_line);
    cv::imshow("horizon line YUD", yud_image);
    cv::waitKey(1);
  }
}

BOOST_AUTO_TEST_CASE(checkEurasianDataset_testCase) {

  std::string gt_path = std::string(EURASIAN_PATH) + "gt_data.yml";

  cv::Mat raw_horizon_lines_gt;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs["horizon_lines"] >> raw_horizon_lines_gt;
  std::cout << " out vpd " << raw_horizon_lines_gt << std::endl;

  cv::Mat1f horizon_lines_gt(raw_horizon_lines_gt);

  std::vector<cv::Point2f> points(3);
  std::vector<cv::Point3f> points_3d(3);

  for (unsigned int k = 0; k < horizon_lines_gt.rows; k++) {
    std::string path_image =
        std::string(EURASIAN_PATH) + numberToString(k + 1, 3) + ".jpg";

    std::cout << " path_image " << path_image << std::endl;
    cv::Mat yud_image = cv::imread(path_image);
    cv::Point3f horizon_line(horizon_lines_gt[k][0], horizon_lines_gt[k][1],
                             horizon_lines_gt[k][2]);

    drawHorizonLine(yud_image, horizon_line);
    cv::imshow("horizon line YUD", yud_image);
    cv::waitKey();
  }
}
