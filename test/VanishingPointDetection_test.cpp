#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VanishingPointDetection_test"

#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <VanishingPointDetection.hpp>
#include <VanishingPointDetectionTools.hpp>
#include <opencv2/highgui/highgui.hpp>

#define YUD_PATH "../../../resource/yud_dataset/"

using namespace vanishing_point;

BOOST_AUTO_TEST_CASE(standardVanishingPoint_testeCase){

  VanishingPointDetection* detector = new VanishingPointDetection();
  std::vector<cv::Point2f> out = detector->applyDetector(cv::Mat());

  BOOST_CHECK_EQUAL(0, out.size());
}

BOOST_AUTO_TEST_CASE(drawOrthogonalVP_testeCase){

  cv::Mat3b temp_image = cv::Mat3b::zeros(500,500);
  std::vector<cv::Point2f> points;
  points.push_back(cv::Point2f(400,230)); //VP X -> RED
  points.push_back(cv::Point2f(100,200)); //VP Y -> GREEN
  points.push_back(cv::Point2f(250,400)); //VP Z -> BLUE

  temp_image = drawOrthogonalVP(temp_image,points,false);

  std::vector<cv::Mat> channels;
  cv::split(temp_image, channels);

  for (unsigned int i = 0; i < points.size(); i++) {
    cv::Point max_point;
    cv::minMaxLoc(channels[2 - i], 0,0, 0, &max_point);
    BOOST_CHECK_CLOSE(points[i].x, max_point.x, 3.0);
    BOOST_CHECK_CLOSE(points[i].y, max_point.y, 3.0);
  }
}

BOOST_AUTO_TEST_CASE(checkYUDDataset_testCase){

  std::string image_path = std::string(YUD_PATH) + "YUD_0000.jpg";
  std::string gt_path = std::string(YUD_PATH) + "vpd_gt.yml";

  cv::Mat yud_image;
  yud_image = cv::imread(image_path);

  cv::Mat raw_gt, camera_params;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs["vpd_gt"] >> raw_gt;
  std::cout << " out vpd " << raw_gt <<std::endl;
  cv::Mat1f yud_gt(raw_gt);

  cv::Point2f size_image(yud_image.cols,yud_image.rows);
  cv::Point2f center_image(size_image.x/2,size_image.y/2);

  std::vector<cv::Point2f> points(3), m_points(3), t_points(3);
  unsigned int k=0;
  points[0] = cv::Point2f(yud_gt[k][0]/yud_gt[k][2],yud_gt[k][1]/yud_gt[k][2]);
  points[1] = cv::Point2f(yud_gt[k][3]/yud_gt[k][5],yud_gt[k][4]/yud_gt[k][5]);
  points[2] = cv::Point2f(yud_gt[k][6]/yud_gt[k][8],yud_gt[k][7]/yud_gt[k][8]);

  m_points[0] = cv::Point2f(points[0].x*size_image.x,points[0].y*size_image.y);
  m_points[1] = cv::Point2f(points[1].x*size_image.x,points[1].y*size_image.y);
  m_points[2] = cv::Point2f(points[2].x*size_image.x,points[2].y*size_image.y);

  t_points[0] = m_points[0]+center_image;
  t_points[1] = m_points[1]+center_image;
  t_points[2] = m_points[2]+center_image;

  std::cout <<"FIRST:"
            <<"\nP0 "<<points[0]<<" "<<m_points[0]<<" "<<t_points[0]
            <<"\nP1 "<<points[1]<<" "<<m_points[1]<<" "<<t_points[1]
            <<"\nP2 "<<points[2]<<" "<<m_points[2]<<" "<<t_points[2]
            <<std::endl;

  // yud_image = drawOrthogonalVP(yud_image,m_points);
  yud_image = drawOrthogonalVP(yud_image,t_points);
  cv::imshow("out YUD FIRST",yud_image);

// second test
  // points[0] = cv::Point2f(yud_gt[k][0],yud_gt[k][1]);
  // points[1] = cv::Point2f(yud_gt[k][3],yud_gt[k][4]);
  // points[2] = cv::Point2f(yud_gt[k][6],yud_gt[k][7]);
  //
  // m_points[0] = cv::Point2f(points[0].x*size_image.x,points[0].y*size_image.y);
  // m_points[1] = cv::Point2f(points[1].x*size_image.x,points[1].y*size_image.y);
  // m_points[2] = cv::Point2f(points[2].x*size_image.x,points[2].y*size_image.y);
  //
  // std::cout <<"SECOND:"
  //           <<"\nP0 "<<points[0]<<" "<<m_points[0]
  //           <<"\nP1 "<<points[1]<<" "<<m_points[1]
  //           <<"\nP2 "<<points[2]<<" "<<m_points[2]
  //           <<std::endl;
  //
  // yud_image = drawOrthogonalVP(yud_image,m_points);
  // cv::imshow("out YUD SECOND",yud_image);
  // cv::waitKey();

  // points[0] = cv::Point2f(points[0].x*size_image.x,points[0].y*size_image.y);
  // points[0] = points[0]+center_image;
  //
  // points[1] = cv::Point2f(points[1].x*size_image.x,points[1].y*size_image.y);
  // points[1] = points[1]+center_image;
  //
  // points[2] = cv::Point2f(points[2].x*size_image.x,points[2].y*size_image.y);
  // points[2] = points[2]+center_image;
  //
  // std::cout<<"P0 "<<points[0]<<"\nP1 "<<points[1]<<"\nP2 "<<points[2]<<std::endl;
  //
  // yud_image = drawOrthogonalVP(yud_image,points);
  // cv::imshow("out YUD",yud_image);
  cv::waitKey();
}
