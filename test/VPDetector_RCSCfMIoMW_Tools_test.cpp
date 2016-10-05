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
#include <vp_detectors/VanishingPointDetectionTools.hpp>

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

  std::vector<double> gt_error = {10, 5, 6, 100, 3.02, 9.0673};

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

  std::vector< std::vector<cv::Vec4f> > set4_segments(2);
  set4_segments[0]= {
              cv::Vec4f(0.6, 0.1, 0.7, 0.1), cv::Vec4f(0.55, 0.2, 0.55, 0.3),
              cv::Vec4f(0.7, 0.4, 0.8, 0.5), cv::Vec4f(0.8, 0.7, 0.7, 0.8)};

  set4_segments[1]= {
              cv::Vec4f(0.3, 0.0, 0.4, 0.0), cv::Vec4f(0.3, 0.2, 0.4, 0.1),
              cv::Vec4f(0.1, 0.7, 0.1, 0.8), cv::Vec4f(0.2, 0.9, 0.3, 0.9)};

  for (uint k = 0; k < set4_segments.size(); k++) {
    std::vector<cv::Vec4f> line_segments = set4_segments[k];

    for (uint i = 0; i < line_segments.size(); i++) {
      cv::Mat temp_mat = cv::Mat(line_segments).row(i);
      for (uint j = 0; j < 2; j++) {
        cv::Mat1f local_mat(temp_mat);
        local_mat[0][j*2] = local_mat[0][j*2]*image.cols - center_point.x;
        local_mat[0][j*2+1] = local_mat[0][j*2+1]*image.rows - center_point.y;
      }
    }

    std::vector<cv::Point3f> lines(4);
    for (uint i = 0; i < line_segments.size(); i++) {
      cv::Point2f initial_point(line_segments[i][0], line_segments[i][1]);
      cv::Point2f end_point(line_segments[i][2], line_segments[i][3]);
      lines[i] = defineEuclidianLineBy2Points(initial_point, end_point);
    }

    std::vector<cv::Point2f> vps = estimationVPby4LinesCase1(lines);

    cv::Point2f vp3 = checkVPTriangle(vps[0], vps[1], cv::Point2f(0,0));

    BOOST_CHECK_CLOSE(vp3.x, vps[2].x, 0.001);
    BOOST_CHECK_CLOSE(vp3.y, vps[2].y, 0.001);


    // visual debug
    // std::cout << "VPS FINAL " <<  cv::Mat(vps) << std::endl;
    // std::cout << "VP Check " << vp3 << std::endl;

    // for (uint i = 0; i < line_segments.size(); i++) {
    //   cv::Point2f point1(line_segments[i][0],line_segments[i][1]);
    //   cv::Point2f point2(line_segments[i][2],line_segments[i][3]);
    //   cv::line( image, point1 + center_point, point2 + center_point,
    //             cv::Scalar(0, 255, 255), image.rows * 0.002);
    // }
    //
    // cv::circle( image, center_point, image.rows * 0.005,
    //             cv::Scalar(255,255,255), -1);
    // for (uint i = 0; i < vps.size(); i++) {
    //   cv::circle( image, vps[i] +center_point, image.rows * 0.005,
    //               cv::Scalar(0,255,0), -1);
    // }
    //
    // cv::imshow("out 1", image);
    // cv::waitKey();
    // image = cv::Mat3b::zeros(500,500);
  }
}

BOOST_AUTO_TEST_CASE(estimationVPby4LinesCase2_testCase){

  cv::Mat3b image = cv::Mat3b::zeros(500,500);
  cv::Point2f center_point(image.cols/2, image.rows/2);

  std::vector< std::vector<cv::Vec4f> > set4_segments(2);
  set4_segments[0]= {
              cv::Vec4f(0.6, 0.2, 0.7, 0.2), cv::Vec4f(0.5, 0.2, 0.5, 0.3),
              cv::Vec4f(0.8, 0.7, 0.7, 0.69), cv::Vec4f(0.2, 0.7, 0.3, 0.69)};
  set4_segments[1]= {
              cv::Vec4f(0.3, 0.0, 0.4, 0.0), cv::Vec4f(0.3, 0.2, 0.4, 0.1),
              cv::Vec4f(0.8, 0.9, 0.7, 0.88), cv::Vec4f(0.1, 0.9, 0.2, 0.899)};

  for (uint k = 0; k < set4_segments.size(); k++) {
    std::vector<cv::Vec4f> line_segments = set4_segments[k];

    for (uint i = 0; i < line_segments.size(); i++) {
      cv::Mat temp_mat = cv::Mat(line_segments).row(i);
      for (uint j = 0; j < 2; j++) {
        cv::Mat1f local_mat(temp_mat);
        local_mat[0][j*2] = local_mat[0][j*2]*image.cols - center_point.x;
        local_mat[0][j*2+1] = local_mat[0][j*2+1]*image.rows - center_point.y;
      }
    }

    std::vector<cv::Point3f> lines(4);
    for (uint i = 0; i < line_segments.size(); i++) {
      cv::Point2f initial_point(line_segments[i][0], line_segments[i][1]);
      cv::Point2f end_point(line_segments[i][2], line_segments[i][3]);
      lines[i] = defineEuclidianLineBy2Points(initial_point, end_point);
    }

    std::vector<cv::Point2f> vps = estimationVPby4LinesCase2(lines);
    cv::Point2f vp3 = checkVPTriangle(vps[0], vps[1], cv::Point2f(0,0));

    BOOST_CHECK_CLOSE(vp3.x, vps[2].x, 0.01);
    BOOST_CHECK_CLOSE(vp3.y, vps[2].y, 0.01);

    // DEBUG
    // std::cout << "VPS FINAL " <<  cv::Mat(vps) << std::endl;
    // std::cout << "VP Check " << vp3 << std::endl;
    //
    // for (uint i = 0; i < line_segments.size(); i++) {
    //   cv::Point2f point1(line_segments[i][0],line_segments[i][1]);
    //   cv::Point2f point2(line_segments[i][2],line_segments[i][3]);
    //   cv::line( image, point1 + center_point, point2 + center_point,
    //             cv::Scalar(0, 255, 255), image.rows * 0.002);
    // }
    //
    // cv::circle( image, center_point, image.rows * 0.005,
    //             cv::Scalar(255,255,255), -1);
    //
    // for (uint i = 0; i < vps.size(); i++)
    //   cv::line( image, vps[i%3] + center_point, vps[(i+1)%3] + center_point,
    //             cv::Scalar(0, 255), image.rows * 0.008);
    //
    //
    // std::vector<cv::Point2f> vps_check = {vps[0], vps[1], vp3};
    // for (uint i = 0; i < vps.size(); i++)
    //   cv::line( image, vps_check[i%3] + center_point, vps_check[(i+1)%3] + center_point,
    //             cv::Scalar(255, 255), image.rows * 0.002);
    //
    //
    // cv::imshow("out", image);
    // cv::waitKey();
    // image = cv::Mat3b::zeros(500,500);

  }

}

BOOST_AUTO_TEST_CASE(estimationVPby4LinesInAll9Cases_testCase){

  cv::Mat3b image = cv::Mat3b::zeros(500,500);
  cv::Point2f center_point(image.cols/2, image.rows/2);
  std::vector< std::vector<double> > gt_foco(2);
  gt_foco[0] = {220.7940393500241, -1, 136.9306465080955, 170.5130470631207,
                -1, 51.34580650552803, 161.7431171255828, -1, -1};
  gt_foco[1] = {-1, 112.9158979063621, 112.9158979063621, 123.0318810308938,
                2345.207879911715, 0.2059347336891552, -1, -1, -1};

  std::vector< std::vector<cv::Vec4f> > set4_segments(2);
  set4_segments[0]= {
              cv::Vec4f(0.6, 0.1, 0.7, 0.1), cv::Vec4f(0.55, 0.2, 0.55, 0.3),
              cv::Vec4f(0.7, 0.4, 0.8, 0.5), cv::Vec4f(0.4, 0.6, 0.6, 0.7)};
  set4_segments[1]= {
              cv::Vec4f(0.6, 0.2, 0.7, 0.2), cv::Vec4f(0.5, 0.2, 0.5, 0.3),
              cv::Vec4f(0.8, 0.7, 0.7, 0.69), cv::Vec4f(0.2, 0.7, 0.3, 0.69)};


  for (uint k = 0; k < set4_segments.size(); k++) {

    std::vector<cv::Vec4f> line_segments = set4_segments[k];

    for (uint i = 0; i < line_segments.size(); i++) {
      cv::Mat temp_mat = cv::Mat(line_segments).row(i);
      for (uint j = 0; j < 2; j++) {
        cv::Mat1f local_mat(temp_mat);
        local_mat[0][j*2] = local_mat[0][j*2]*image.cols - center_point.x;
        local_mat[0][j*2+1] = local_mat[0][j*2+1]*image.rows - center_point.y;
      }
    }

    std::vector<cv::Point3f> lines(4);
    for (uint i = 0; i < line_segments.size(); i++) {
      cv::Point2f initial_point(line_segments[i][0], line_segments[i][1]);
      cv::Point2f end_point(line_segments[i][2], line_segments[i][3]);
      lines[i] = defineEuclidianLineBy2Points(initial_point, end_point);
    }

    std::vector< double > vector_foco;
    std::vector< std::vector<cv::Point2f> > vector_vps;
    vector_vps = estimationVPby4LinesInAll9Cases(lines, &vector_foco);

    // std::cout << cv::Mat(vector_foco).t() <<std::endl;
    for (uint i = 0; i < vector_foco.size(); i++) {
      BOOST_CHECK_CLOSE(vector_foco[i], gt_foco[k][i], 0.01);
    }
    // for (uint i = 0; i < vector_foco.size(); i++) {
    //   std::cout << "Case "<< i <<" Foco "<< vector_foco[i] << " Mat "
    //             <<  cv::Mat(vector_vps[i]) << std::endl;
    // }

    std::cout << std::endl;
  }

}

BOOST_AUTO_TEST_CASE(isPointLaySegmentLine_testCase){

  std::vector<cv::Vec4f> segments = { cv::Vec4f(-3,-3,3,1),
                                      cv::Vec4f(-1,8,1,2),
                                      cv::Vec4f(-3,-3,3,1),
                                      cv::Vec4f(-1,8,1,2),
                                      cv::Vec4f(0,0,2,2),
                                      cv::Vec4f(0,0,2,2)};

  std::vector<cv::Point2f> points = {cv::Point2f(0,-1), cv::Point2f(0,5),
                                     cv::Point2f(0,-0.999), cv::Point2f(0,0),
                                     cv::Point2f(1,1), cv::Point2f(3,3)};

  std::vector<bool> gts = {true, true, false, false, true, false};

  for (uint i = 0; i < points.size(); i++) {
    bool result = isPointLaySegmentLine(points[i], segments[i]);
    BOOST_CHECK_EQUAL(gts[i], result);
    // std::cout << " Point lay on segment " << i << " " << result << std::endl;
  }

}


BOOST_AUTO_TEST_CASE(filterHypotheses_testCase){

  cv::Mat3b image = cv::Mat3b::zeros(500,500);
  cv::Point2f center_point(image.cols/2, image.rows/2);
  std::vector< std::vector<cv::Vec4f> > set4_segments(4);
  set4_segments[0]= {
              cv::Vec4f(0.6, 0.1, 0.7, 0.1), cv::Vec4f(0.55, 0.2, 0.55, 0.3),
              cv::Vec4f(0.7, 0.4, 0.8, 0.5), cv::Vec4f(0.4, 0.6, 0.6, 0.7),
              cv::Vec4f(0.1, 0.8, 0.245, 0.534)};
  set4_segments[1]= {
              cv::Vec4f(0.6, 0.05, 0.7, 0.05), cv::Vec4f(0.5, 0.2, 0.5, 0.3),
              cv::Vec4f(0.8, 0.7, 0.7, 0.69), cv::Vec4f(0.2, 0.7, 0.3, 0.69),
              cv::Vec4f(0.942, 0.467, 0.712, 0.11)};
  set4_segments[2]= {
              cv::Vec4f(0.3, 0.0, 0.4, 0.0), cv::Vec4f(0.3, 0.2, 0.4, 0.1),
              cv::Vec4f(0.8, 0.9, 0.7, 0.88), cv::Vec4f(0.1, 0.9, 0.2, 0.899),
              cv::Vec4f(0.987, 0.423, 0.481, 0.8853)};
  set4_segments[3]= {
              cv::Vec4f(0.3, 0.0, 0.4, 0.0), cv::Vec4f(0.3, 0.2, 0.4, 0.1),
              cv::Vec4f(0.1, 0.7, 0.1, 0.8), cv::Vec4f(0.2, 0.9, 0.3, 0.9),
              cv::Vec4f(0.98, 1.0, 1.0, 1.0)};

  std::vector< std::vector<double> > gt_foco(4);
  gt_foco[0] = {220};
  gt_foco[1] = {138, 138, 152};
  gt_foco[2] = {221.73};
  gt_foco[3] = {223};


  for (uint k = 0; k < set4_segments.size(); k++) {

    std::vector<cv::Vec4f> line_segments = set4_segments[k];

    for (uint i = 0; i < line_segments.size(); i++) {
      cv::Mat temp_mat = cv::Mat(line_segments).row(i);
      for (uint j = 0; j < 2; j++) {
        cv::Mat1f local_mat(temp_mat);
        local_mat[0][j*2] = local_mat[0][j*2]*image.cols - center_point.x;
        local_mat[0][j*2+1] = local_mat[0][j*2+1]*image.rows - center_point.y;
      }
    }

    std::vector<cv::Point3f> lines(4);
    for (uint i = 0; i < line_segments.size()-1; i++) {
      cv::Point2f initial_point(line_segments[i][0], line_segments[i][1]);
      cv::Point2f end_point(line_segments[i][2], line_segments[i][3]);
      lines[i] = defineEuclidianLineBy2Points(initial_point, end_point);
    }

    std::vector< double > vector_foco;
    std::vector< std::vector<cv::Point2f> > vector_vps;
    vector_vps = estimationVPby4LinesInAll9Cases(lines, &vector_foco);
    vector_vps = filterHypotheses(vector_vps, vector_foco, line_segments);


    for (uint i = 0; i < vector_foco.size(); i++) {
      BOOST_CHECK_CLOSE(gt_foco[k][i], vector_foco[i], 1.0);
    }


    // DEBUG
    // std::cout << "FINAL FOCO " <<  cv::Mat(vector_foco).t() << std::endl;
    //
    // for (uint j = 0; j < vector_vps.size(); j++) {
    //
    //   for (uint i = 0; i < line_segments.size(); i++) {
    //     cv::Point2f point1(line_segments[i][0],line_segments[i][1]);
    //     cv::Point2f point2(line_segments[i][2],line_segments[i][3]);
    //     cv::line( image, point1 + center_point, point2 + center_point,
    //               cv::Scalar(0, 255, 255), image.rows * 0.002);
    //   }
    //
    //   cv::circle( image, center_point, image.rows * 0.005,
    //               cv::Scalar(255,255,255), -1);
    //
    //
    //   for (uint i = 0; i < vector_vps[j].size(); i++){
    //     cv::line( image, vector_vps[j][i%3] + center_point,
    //               vector_vps[j][(i+1)%3] + center_point,
    //               cv::Scalar(0, 255), image.rows * 0.008);
    //   }
    // cv::imshow("out", image);
    // cv::waitKey();
    // image = cv::Mat3b::zeros(500,500);
    // }
  }
}

BOOST_AUTO_TEST_CASE(consensusSet_testCase){

  cv::Mat3b image = cv::Mat3b::zeros(1000,1000);
  cv::Point2f center_point(image.cols/2, image.rows/2);
  std::vector<cv::Vec4f> segments = {  cv::Vec4f(0.4, 0, 0.45, 0.05),
          cv::Vec4f(0.6, 0.05, 0.65, 0),    cv::Vec4f(0.15, 0.1, 0.2, 0.1),
          cv::Vec4f(0.8, 0.1, 0.85, 0.1),   cv::Vec4f(0.45, 0.2, 0.5, 0.15),
          cv::Vec4f(0.65, 0.15, 0.7, 0.2),  cv::Vec4f(0.15, 0.3, 0.2, 0.35),
          cv::Vec4f(0.05, 0.5, 0.1, 0.5),   cv::Vec4f(0.4, 0.45, 0.45, 0.4),
          cv::Vec4f(0.6, 0.5, 0.65, 0.5),   cv::Vec4f(0.35, 0.6, 0.35, 0.65),
          cv::Vec4f(0.6, 0.75, 0.65, 0.8),  cv::Vec4f(0.9, 0, 0.9, 0.05),
          cv::Vec4f(0.8, 0.5, 0.85, 0.55),  cv::Vec4f(0.95, 0.55, 1.0, 0.5),
          cv::Vec4f(0.95, 0.65, 1.0, 0.7), cv::Vec4f(0.75, 0.75, 0.8, 0.7),
          cv::Vec4f(0.9, 0.95, 0.9, 1.0)};

  std::vector<cv::Point2f> vps = {cv::Point2f(0.55, 0.1), cv::Point2f(0.9, 0.6),
                                  cv::Point2f(0.35, 0.5)};

  std::vector<int> gt_cluster = {-1, 0, 0, 0, 0, -1, 2, 2, 2, 2, 2,
                                  2, 1, 1, 1, 1, 1, 1};

  std::vector<int> lines_cluster;
  uint count = consensusSet(vps, segments, lines_cluster, 0.0001);

  for (uint i = 0; i < lines_cluster.size(); i++)
    BOOST_CHECK_EQUAL(gt_cluster[i], lines_cluster[i]);

  // DEBUG
  // std::cout<<cv::Mat(lines_cluster).t()<<std::endl;
  // for (uint i = 0; i < segments.size(); i++) {
  //   cv::Scalar color (255, 255, 255);
  //   if(lines_cluster[i] == 0){
  //     color =  cv::Scalar(255, 0, 0);
  //   }else if(lines_cluster[i] == 1){
  //     color =  cv::Scalar(0, 255, 0);
  //   }else if(lines_cluster[i] == 2){
  //     color =  cv::Scalar(0, 0, 255);
  //   }
  //   cv::line( image,
  //           cv::Point2f(segments[i][0]*image.cols, segments[i][1]*image.rows),
  //           cv::Point2f(segments[i][2]*image.cols, segments[i][3]*image.rows),
  //           color, image.rows * 0.004);
  // }
  // for (uint i = 0; i < vps.size(); i++)
  //   cv::circle( image, vps[i] * image.cols, image.rows * 0.004, cv::Scalar(255,255), -1);
  //   cv::imshow("out", image);
  //
  // cv::waitKey();
}




BOOST_AUTO_TEST_CASE(RANSAC_controled_testCase){

  cv::Mat3b image = cv::Mat3b::zeros(1000,1000);
  cv::Point2f center_point(image.cols/2, image.rows/2);

  std::vector< std::vector<cv::Vec4f> > vector_segments(3);

  vector_segments[0] = {  cv::Vec4f(0.4, 0, 0.45, 0.05),
          cv::Vec4f(0.6, 0.05, 0.65, 0),    cv::Vec4f(0.15, 0.1, 0.2, 0.1),
          cv::Vec4f(0.8, 0.1, 0.85, 0.1),   cv::Vec4f(0.45, 0.2, 0.5, 0.15),
          cv::Vec4f(0.65, 0.15, 0.7, 0.2),  cv::Vec4f(0.15, 0.3, 0.2, 0.35),
          cv::Vec4f(0.05, 0.5, 0.1, 0.5),   cv::Vec4f(0.4, 0.45, 0.45, 0.4),
          cv::Vec4f(0.6, 0.5, 0.65, 0.5),   cv::Vec4f(0.35, 0.6, 0.35, 0.65),
          cv::Vec4f(0.6, 0.75, 0.65, 0.8),  cv::Vec4f(0.9, 0, 0.9, 0.05),
          cv::Vec4f(0.8, 0.5, 0.85, 0.55),  cv::Vec4f(0.95, 0.55, 1.0, 0.5),
          cv::Vec4f(0.95, 0.65, 1.0, 0.7), cv::Vec4f(0.75, 0.75, 0.8, 0.7),
          cv::Vec4f(0.9, 0.95, 0.9, 1.0)};

  vector_segments[1] = {  cv::Vec4f(0.4, 0, 0.45, 0.05),
          cv::Vec4f(0.6, 0.05, 0.65, 0),    cv::Vec4f(0.15, 0.1, 0.2, 0.1),
          cv::Vec4f(0.8, 0.1, 0.85, 0.1),   cv::Vec4f(0.45, 0.2, 0.5, 0.15),
          cv::Vec4f(0.65, 0.15, 0.7, 0.2),  cv::Vec4f(0.15, 0.3, 0.2, 0.35),
          cv::Vec4f(0.05, 0.5, 0.1, 0.5),   cv::Vec4f(0.4, 0.45, 0.45, 0.4),
          cv::Vec4f(0.6, 0.5, 0.65, 0.5),   cv::Vec4f(0.35, 0.6, 0.35, 0.65),
          cv::Vec4f(0.6, 0.75, 0.65, 0.8),  cv::Vec4f(0.9, 0, 0.9, 0.05),
          cv::Vec4f(0.8, 0.5, 0.85, 0.55),  cv::Vec4f(0.95, 0.54, 1.0, 0.5),
          cv::Vec4f(0.94, 0.65, 1.0, 0.7),  cv::Vec4f(0.75, 0.74, 0.8, 0.7),
          cv::Vec4f(0.91, 0.95, 0.9, 1.0)};

  vector_segments[2] = {  cv::Vec4f(0.4, 0, 0.45, 0.05),
          cv::Vec4f(0.6, 0.05, 0.65, 0),    cv::Vec4f(0.15, 0.1, 0.2, 0.1),
          cv::Vec4f(0.8, 0.1, 0.85, 0.1),   cv::Vec4f(0.45, 0.2, 0.5, 0.15),
          cv::Vec4f(0.65, 0.15, 0.7, 0.2),  cv::Vec4f(0.14, 0.3, 0.2, 0.35),
          cv::Vec4f(0.05, 0.49, 0.1, 0.5),  cv::Vec4f(0.41, 0.45, 0.45, 0.4),
          cv::Vec4f(0.6, 0.5, 0.65, 0.51),  cv::Vec4f(0.34, 0.6, 0.35, 0.65),
          cv::Vec4f(0.6, 0.75, 0.65, 0.81),  cv::Vec4f(0.9, 0, 0.9, 0.05),
          cv::Vec4f(0.8, 0.5, 0.85, 0.55),  cv::Vec4f(0.95, 0.55, 1.0, 0.5),
          cv::Vec4f(0.95, 0.65, 1.0, 0.7),  cv::Vec4f(0.75, 0.75, 0.8, 0.7),
          cv::Vec4f(0.9, 0.95, 0.9, 1.0)};

  std::vector<cv::Point2f> gt_vp = {cv::Point2f(400, -2200),
        cv::Point2f(50, 25), cv::Point2f(-60.606, 42.424)};


  for (uint k = 0; k < vector_segments.size(); k++) {

    std::vector<cv::Vec4f> segments = vector_segments[k];

    for (uint i = 0; i < segments.size(); i++) {
      cv::Mat temp_mat = cv::Mat(segments).row(i);
      for (uint j = 0; j < 2; j++) {
        cv::Mat1f local_mat(temp_mat);
        local_mat[0][j*2] = local_mat[0][j*2]*image.cols - center_point.x;
        local_mat[0][j*2+1] = local_mat[0][j*2+1]*image.rows - center_point.y;
      }
    }

    std::vector<cv::Point3f> lines;
    for (uint i = 0; i < segments.size()-1; i++) {
      cv::Point2f initial_point(segments[i][0], segments[i][1]);
      cv::Point2f end_point(segments[i][2], segments[i][3]);
      lines.push_back(defineEuclidianLineBy2Points(initial_point, end_point));
    }

    std::vector<int> lines_cluster;
    std::vector<cv::Point2f> vps = RANSAC(segments, lines, 250, lines_cluster);

    BOOST_CHECK_CLOSE(gt_vp[k].x, vps[2].x, 0.01);
    BOOST_CHECK_CLOSE(gt_vp[k].y, vps[2].y, 0.01);

    // std::cout << cv::Mat(lines_cluster).t() << std::endl;
    // for (uint i = 0; i < segments.size(); i++) {
    //   cv::Scalar color (255, 255, 255);
    //   if(lines_cluster[i] == 0){
    //     color =  cv::Scalar(255, 0, 0);
    //   }else if(lines_cluster[i] == 1){
    //     color =  cv::Scalar(0, 255, 0);
    //   }else if(lines_cluster[i] == 2){
    //     color =  cv::Scalar(0, 0, 255);
    //   }
    //   cv::line( image,
    //           cv::Point2f(segments[i][0] + center_point.x, segments[i][1] + center_point.y),
    //           cv::Point2f(segments[i][2] + center_point.x, segments[i][3] + center_point.y),
    //           color, image.rows * 0.004);
    // }
    //
    // std::cout << cv::Mat(vps) << std::endl;
    // cv::Point2f vp_check = checkVPTriangle(vps[0], vps[1], cv::Point2f(0,0));
    // std::cout << vp_check << std::endl;
    //
    // for (uint i = 0; i < vps.size(); i++)
    //   cv::circle( image, vps[i] + center_point, image.rows * 0.004, cv::Scalar(255,255), -1);
    //
    // cv::imshow("out", image);
    // cv::waitKey();
    // image = cv::Mat3b::zeros(1000,1000);
  }
}
