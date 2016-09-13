#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VanishingPointDetectionTools_test"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

// #include <VanishingPointDetection.hpp>
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

// BOOST_AUTO_TEST_CASE(standardVanishingPoint_testeCase) {
//
//   VanishingPointDetection *detector = new VanishingPointDetection();
//   std::vector<cv::Point2f> out = detector->applyVPDetector(cv::Mat());
//
//   BOOST_CHECK_EQUAL(0, out.size());
// }

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

BOOST_AUTO_TEST_CASE(chechAdjustPointsToDraw_testCase){

  std::vector< cv::Point2f > zenith_vector = {
    cv::Point2f(200, 300), cv::Point2f(330, 200),
    cv::Point2f(250, -10), cv::Point2f(250, 510)};
  std::vector< cv::Point3f > horizon_vector = {
    cv::Point3f(-0.05, -1, 310), cv::Point3f(0.10, -1, 170),
    cv::Point3f(0.08, -1, 400), cv::Point3f(-0.10, -1, 100)};

  std::vector< cv::Point3f > gt_zenith_line = {
    cv::Point3f(-1, -1, 500), cv::Point3f(-0.625, -1, 406.25),
    cv::Point3f(-1, 0, 250), cv::Point3f(-1, 0, 250)};
  std::vector< cv::Point2f > gt_zenith_local = {
    cv::Point2f(200, 300), cv::Point2f(330, 200),
    cv::Point2f(250, 0), cv::Point2f(250, 500)};
  std::vector< cv::Point2f > gt_intersection = {
    cv::Point2f(200, 300), cv::Point2f(325.86, 202.58),
    cv::Point2f(250, 420), cv::Point2f(250, 75)};

  cv::Mat3b image = cv::Mat3b::zeros(500,500);
  cv::Point2f principal_point(image.cols/2, image.rows/2);

  for (int i = 0; i < 4; i++) {
    cv::Point2f intersection_point, zenith_local;
    cv::Point3f zenith_line = adjustPointsToDraw( zenith_vector[i],
      principal_point, horizon_vector[i], &intersection_point, &zenith_local);

    BOOST_CHECK_CLOSE(gt_zenith_line[i].x, zenith_line.x, 0.1);
    BOOST_CHECK_CLOSE(gt_zenith_line[i].y, zenith_line.y, 0.1);
    BOOST_CHECK_CLOSE(gt_zenith_line[i].z, zenith_line.z, 0.1);

    BOOST_CHECK_CLOSE(gt_zenith_local[i].x, zenith_local.x, 0.1);
    BOOST_CHECK_CLOSE(gt_zenith_local[i].y, zenith_local.y, 0.1);

    BOOST_CHECK_CLOSE(gt_intersection[i].x, intersection_point.x, 0.1);
    BOOST_CHECK_CLOSE(gt_intersection[i].y, intersection_point.y, 0.1);

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

    // yud_image = drawHorizonLine(yud_image, horizon_line);
    // cv::imshow("horizon line York Urban", yud_image);
    // cv::waitKey();
  }
}

BOOST_AUTO_TEST_CASE(checkEurasianDataset_testCase) {

  std::string gt_path = std::string(EURASIAN_PATH) + "gt_data.yml";

  cv::Mat raw_horizon_lines_gt, raw_zenith_gt;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs["horizon_lines"] >> raw_horizon_lines_gt;
  fs["zeniths"] >> raw_zenith_gt;
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

    cv::Point2f intersection_point, zenith_local;
    // cv::Point3f zenith_line = adjustPointsToDraw( zenith,
    //                                               principal_point,
    //                                               horizon_line,
    //                                               &intersection_point,
    //                                               &zenith_local);

    // image = drawHorizonLine(image, horizon_line);
    // image = drawZenithLine(image, zenith_local, principal_point, intersection_point);
    // cv::imshow("horizon line Euroasian Cites", image);
    // cv::waitKey();
  }
}

BOOST_AUTO_TEST_CASE(horizonLineEstimation_testCase) {

  cv::Mat3b image = cv::Mat3b::zeros(500,500);
  std::vector<cv::Point2f> zeniths ={
                        cv::Point2f(250,-10), cv::Point2f(250,510),
                        cv::Point2f(200,-10), cv::Point2f(400,510),
                        cv::Point2f(400,-10), cv::Point2f(200,510)};

  std::vector< std::vector<cv::Point2f> > vps;
  vps.push_back({ cv::Point2f(100,400), cv::Point2f(200,350),
                  cv::Point2f(300,310), cv::Point2f(400,410) });
  vps.push_back({ cv::Point2f(100,100), cv::Point2f(200,150),
                  cv::Point2f(300,200), cv::Point2f(400,110) });
  vps.push_back({ cv::Point2f(100,400), cv::Point2f(200,350),
                  cv::Point2f(300,325), cv::Point2f(400,300) });
  vps.push_back({ cv::Point2f(100,350), cv::Point2f(200,300),
                  cv::Point2f(300,250), cv::Point2f(400,200) });
  vps.push_back({ cv::Point2f(100,200), cv::Point2f(200,350),
                  cv::Point2f(300,450), cv::Point2f(400,490) });
  vps.push_back({ cv::Point2f(100,50), cv::Point2f(200,63),
                  cv::Point2f(300,100), cv::Point2f(400,110) });

  std::vector< std::vector<int> > lines_by_vp;
  lines_by_vp.push_back({8, 4, 3, 5});
  lines_by_vp.push_back({3, 5, 2, 9});
  lines_by_vp.push_back({1, 8, 3, 8});
  lines_by_vp.push_back({3, 1, 8, 8});
  lines_by_vp.push_back({2, 4, 6, 8});
  lines_by_vp.push_back({8, 4, 2, 6});

  std::vector<cv::Point3f> gt_horizon_line = {
    cv::Point3f(0, -1, 379), cv::Point3f(0, -1, 128.421),
    cv::Point3f(-0.192308, -1, 384.519), cv::Point3f(-0.576923, -1, 423.462),
    cv::Point3f(0.576923, -1, 247.923), cv::Point3f(0.192308, -1, 31.3692)};

  float total_samples = 20;
  float factor = 4;
  cv::Point2f principal_point(image.cols/2, image.rows/2);
  for (int i = 0; i < zeniths.size(); i++) {
    cv::Point3f horizon_line =
                      horizonLineEstimation(zeniths[i], principal_point,
                                            vps[i], lines_by_vp[i]);

    BOOST_CHECK_CLOSE(horizon_line.x, gt_horizon_line[i].x, 0.01);
    BOOST_CHECK_CLOSE(horizon_line.y, gt_horizon_line[i].y, 0.01);
    BOOST_CHECK_CLOSE(horizon_line.z, gt_horizon_line[i].z, 0.01);

    // Visual Debug
    // cv::Point2f zenith_local, intersection_point;
    // adjustPointsToDraw( zeniths[i], principal_point, horizon_line,
    //                     &intersection_point, &zenith_local);
    //
    // image = drawHorizonLine(image, horizon_line);
    // image = drawZenithLine( image, zenith_local, principal_point,
    //                         intersection_point);
    // for (int j = 0; j < vps[i].size(); j++) {
    //   cv::Scalar color ((255.0/8) * lines_by_vp[i][j], 0, 0);
    //   cv::circle(image, vps[i][j], image.rows * 0.01, color, -1);
    // }
    //
    // cv::imshow("out horizon",image);
    // cv::waitKey();
    // image = cv::Mat3b::zeros(500,500);
  }

}

BOOST_AUTO_TEST_CASE(normalizedMaxDistanceBetweenHorizonLines_testCase){

  std::vector<cv::Point3f> horizon_line1 = {
    cv::Point3f(0, -1, 250), cv::Point3f(0.15, -1, 300),
    cv::Point3f(-0.15, -1, 100), cv::Point3f(-0.15, -1, 100),
    cv::Point3f(0.15, -1, 100)};

  std::vector<cv::Point3f> horizon_line2 = {
    cv::Point3f(0, -1, 300), cv::Point3f(0.15, -1, 400),
    cv::Point3f(-0.15, -1, 200), cv::Point3f(0.15, -1, 400),
    cv::Point3f(-0.15, -1, 400)};

  std::vector<double> distances = {0.1, 0.2, 0.2, 0.9, 0.6};
  std::vector<cv::Point2f> point1s = {  cv::Point2f(500,250),
                  cv::Point2f(500,375), cv::Point2f(500,25),
                  cv::Point2f(500,25),  cv::Point2f(0,100)};
  std::vector<cv::Point2f> point2s = {  cv::Point2f(500,300),
                  cv::Point2f(500,475), cv::Point2f(500,125),
                  cv::Point2f(500,475), cv::Point2f(0,400)};

  for (int i = 0; i < horizon_line1.size(); i++) {
    cv::Mat3b image = cv::Mat3b::zeros(500,500);
    cv::Point2f point1, point2;
    double distance = normalizedMaxDistanceBetweenHorizonLines(
      horizon_line1[i], horizon_line2[i], image.size(), &point1, &point2);

    BOOST_CHECK_CLOSE(distance, distances[i], 0.01);
    BOOST_CHECK_CLOSE(point1.x, point1s[i].x, 0.01);
    BOOST_CHECK_CLOSE(point1.y, point1s[i].y, 0.01);
    BOOST_CHECK_CLOSE(point2.x, point2s[i].x, 0.01);
    BOOST_CHECK_CLOSE(point2.y, point2s[i].y, 0.01);

    // visual debug
    // image = drawHorizonLine(image, horizon_line1[i]);
    // image = drawHorizonLine(image, horizon_line2[i], cv::Scalar(0,255,0));
    // cv::line( image, point1, point2,
    //           cv::Scalar(0, 255, 255), image.rows * 0.004);
    // cv::imshow("out image", image);
    //
    // std::cout.precision(5);
    // std::cout << " value "
    //           << std::fixed << distance << std::endl;
    // std::cout << " out "<< point1 << " " << point2 << std::endl;
    //
    // cv::waitKey();
  }
}

BOOST_AUTO_TEST_CASE(MACRO_TIME_COUNT_testCase){

  std::vector<double> time_session(2,0), cumulate_time(2,0),
                      gt_time_session(2,0);

  int times = 1000000;
  for (size_t i = 0; i < times; i++) {
    TIME_COUNT(time_session[0]){
      clock_t start_time = std::clock();
      gt_time_session[0] += double( std::clock() - start_time )/CLOCKS_PER_SEC;
    }
    cumulate_time[0] += time_session[0];

    TIME_COUNT(time_session[1]){
      clock_t start_time = std::clock();
      gt_time_session[1] += double( std::clock() - start_time )/CLOCKS_PER_SEC;
    }
    cumulate_time[1] += time_session[1];
  }

  double epslon = 3.1e-7;
  BOOST_CHECK_CLOSE(cumulate_time[0]/times - gt_time_session[0]/times,
                    epslon , 5);
  BOOST_CHECK_CLOSE(cumulate_time[1]/times - gt_time_session[1]/times,
                    epslon , 5);
}
