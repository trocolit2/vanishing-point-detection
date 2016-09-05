#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VanishingPointDetectionEvaluation_test"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

// #include <VanishingPointDetection.hpp>
#include <VanishingPointDetectionEvaluation.hpp>

#define YUD_PATH "../../../resource/datasets/yorkurban/"
#define EURASIAN_PATH "../../../resource/datasets/eurasiancities/"

using namespace vanishing_point;


class MOCK_VPD : public VanishingPointDetection{

public:
  MOCK_VPD(){};
  std::vector<cv::Point2f> applyVPDetector(
                                cv::Mat image,
                                std::vector<int> *line_id_by_vp = 0,
                                std::vector<cv::Vec4f> *lines_segments = 0 ){

    float width = image.cols;
    float height = image.rows;

    // mock detected vanishing point
    std::vector<cv::Point2f> out_points = {
      cv::Point2f(width*0.2, height*0.9), cv::Point2f(width*0.45, height*1.1),
      cv::Point2f(width*0.4, height*0.8), cv::Point2f(width*0.6, height*0.6),
      cv::Point2f(width*1.2, height*0.2) };

    // mock line segments
    (*lines_segments) = {
        cv::Vec4f(out_points[0].x, out_points[0].y, 100,100),
        cv::Vec4f(out_points[0].x, out_points[0].y, 30,40),
        cv::Vec4f(out_points[0].x, out_points[0].y, 120,200),
        cv::Vec4f(out_points[0].x, out_points[0].y, 120,200),
        cv::Vec4f(out_points[1].x, out_points[1].y, 250,250),
        cv::Vec4f(out_points[1].x, out_points[1].y, 400,300),
        cv::Vec4f(out_points[1].x, out_points[1].y, 200,500),
        cv::Vec4f(out_points[2].x, out_points[2].y, 350,450),
        cv::Vec4f(out_points[2].x, out_points[2].y, 500,300),
        cv::Vec4f(out_points[3].x, out_points[3].y, 400,400),
        cv::Vec4f(out_points[3].x, out_points[3].y, 10,10),
        cv::Vec4f(out_points[3].x, out_points[3].y, 500,500),
        cv::Vec4f(out_points[4].x, out_points[4].y, 600,100),
        cv::Vec4f(out_points[4].x, out_points[4].y, 300,700)};

    // cluster lines by vp
    (*line_id_by_vp) = {0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4};

    return out_points;
  };

};


BOOST_AUTO_TEST_CASE(runEvaluation_testCase){

  VanishingPointDetection *mock_detector = new MOCK_VPD();
  VanishingPointDetectionEvaluation yud_evaluation("YUD", YUD_PATH);
  // yud_evaluation.runEvaluation(mock_detector);

  free(mock_detector);
}


BOOST_AUTO_TEST_CASE(loadGroundTruth_testCase){


  std::vector<cv::Point2f> gt_zeniths =
                                {cv::Point2f(607.228, -16572.6),
                                 cv::Point2f(519.135, -8432.85)};
  std::vector<cv::Point3f> gt_horizonlines =
                            {cv::Point3f(-0.0178457, 1, -269.868),
                             cv::Point3f(0.0120536, -0.999927, 567.297)};

  std::vector<int> gt_size = {102, 103};

  int reference_index = 10;
  cv::Point3f temp_line;
  cv::Point2f temp_point;
  VanishingPointDetectionEvaluation yud_evaluation("YUD", YUD_PATH);
  BOOST_CHECK_EQUAL(yud_evaluation.getGTZeniths().size(), gt_size[0]);
  BOOST_CHECK_EQUAL(yud_evaluation.getGTHorizonLines().size(), gt_size[0]);

  temp_point = yud_evaluation.getGTZeniths()[reference_index];
  temp_line = yud_evaluation.getGTHorizonLines()[reference_index];
  BOOST_CHECK_CLOSE(temp_point.x, gt_zeniths[0].x, 0.01);
  BOOST_CHECK_CLOSE(temp_point.y, gt_zeniths[0].y, 0.01);

  BOOST_CHECK_CLOSE(temp_line.x, gt_horizonlines[0].x, 0.01);
  BOOST_CHECK_CLOSE(temp_line.y, gt_horizonlines[0].y, 0.01);
  BOOST_CHECK_CLOSE(temp_line.z, gt_horizonlines[0].z, 0.01);

  VanishingPointDetectionEvaluation eurasian_evaluation("EURASIAN",
                                                        EURASIAN_PATH);

  BOOST_CHECK_EQUAL(eurasian_evaluation.getGTZeniths().size(), gt_size[1]);
  BOOST_CHECK_EQUAL(eurasian_evaluation.getGTHorizonLines().size(), gt_size[1]);

  temp_point = eurasian_evaluation.getGTZeniths()[reference_index];
  temp_line = eurasian_evaluation.getGTHorizonLines()[reference_index];
  BOOST_CHECK_CLOSE(temp_point.x, gt_zeniths[1].x, 0.01);
  BOOST_CHECK_CLOSE(temp_point.y, gt_zeniths[1].y, 0.01);

  BOOST_CHECK_CLOSE(temp_line.x, gt_horizonlines[1].x, 0.01);
  BOOST_CHECK_CLOSE(temp_line.y, gt_horizonlines[1].y, 0.01);
  BOOST_CHECK_CLOSE(temp_line.z, gt_horizonlines[1].z, 0.01);
  }
