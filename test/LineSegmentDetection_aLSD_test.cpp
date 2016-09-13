#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "LineSegmentDetection_aLSD_test"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <line_detectors/aLSD.hpp>

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



BOOST_AUTO_TEST_CASE(applyLSDetector_testCase){

  std::vector<cv::Vec4f> gt_ls_yud = { cv::Vec4f(165, 4, 183, 10),
      cv::Vec4f(89, 259, 101, 260), cv::Vec4f(404, 188, 404, 155), cv::Vec4f(585, 91, 614, 82),
      cv::Vec4f(16, 196, 18, 171), cv::Vec4f(19, 170, 22, 131), cv::Vec4f(373, 234, 391, 237),
      cv::Vec4f(15, 204, 18, 194), cv::Vec4f(410, 160, 411, 188), cv::Vec4f(578, 28, 611, 16),
      cv::Vec4f(522, 365, 521, 338), cv::Vec4f(579, 34, 611, 24), cv::Vec4f(149, 2, 162, 7),
      cv::Vec4f(582, 84, 614, 75), cv::Vec4f(581, 63, 613, 53), cv::Vec4f(121, 340, 146, 341),
      cv::Vec4f(579, 42, 610, 31), cv::Vec4f(119, 320, 96, 319), cv::Vec4f(473, 361, 476, 379),
      cv::Vec4f(624, 200, 594, 204), cv::Vec4f(24, 120, 26, 95), cv::Vec4f(582, 105, 599, 102),
      cv::Vec4f(136, 14, 149, 2), cv::Vec4f(561, 239, 565, 252), cv::Vec4f(551, 330, 549, 249),
      cv::Vec4f(23, 131, 23, 121), cv::Vec4f(584, 24, 583, 7), cv::Vec4f(460, 288, 460, 255),
      cv::Vec4f(505, 135, 502, 99), cv::Vec4f(35, 150, 38, 138), cv::Vec4f(108, 436, 111, 419),
      cv::Vec4f(571, 371, 571, 383), cv::Vec4f(575, 134, 599, 132), cv::Vec4f(541, 381, 539, 339),
      cv::Vec4f(28, 343, 41, 343), cv::Vec4f(549, 244, 549, 219), cv::Vec4f(458, 356, 458, 336),
      cv::Vec4f(472, 336, 472, 358), cv::Vec4f(460, 375, 459, 358), cv::Vec4f(106, 31, 127, 11),
      cv::Vec4f(26, 94, 32, 76), cv::Vec4f(483, 209, 484, 181), cv::Vec4f(310, 344, 339, 345),
      cv::Vec4f(173, 63, 186, 54), cv::Vec4f(485, 290, 485, 274), cv::Vec4f(37, 136, 40, 120),
      cv::Vec4f(601, 365, 600, 353), cv::Vec4f(529, 296, 530, 280), cv::Vec4f(85, 376, 92, 278),
      cv::Vec4f(474, 336, 538, 337) };

  std::vector<cv::Vec4f> gt_ls_eurasian ={cv::Vec4f(345, 935, 279, 939),
      cv::Vec4f(300, 935, 328, 933), cv::Vec4f(641, 725, 719, 702), cv::Vec4f(720, 702, 739, 697),
      cv::Vec4f(78, 1095, 166, 1090), cv::Vec4f(624, 400, 614, 381), cv::Vec4f(719, 740, 718, 759),
      cv::Vec4f(185, 1027, 220, 1029), cv::Vec4f(581, 407, 577, 396), cv::Vec4f(540, 213, 579, 148),
      cv::Vec4f(305, 1011, 294, 1015), cv::Vec4f(499, 193, 488, 194), cv::Vec4f(235, 705, 203, 712),
      cv::Vec4f(616, 379, 625, 396), cv::Vec4f(670, 0, 617, 92), cv::Vec4f(506, 266, 521, 242),
      cv::Vec4f(279, 936, 299, 936), cv::Vec4f(340, 922, 333, 910), cv::Vec4f(878, 397, 894, 402),
      cv::Vec4f(584, 141, 642, 41), cv::Vec4f(394, 536, 419, 529), cv::Vec4f(193, 1102, 95, 1110),
      cv::Vec4f(527, 10, 539, 18), cv::Vec4f(518, 253, 510, 264), cv::Vec4f(336, 683, 314, 688),
      cv::Vec4f(221, 1030, 264, 1030), cv::Vec4f(738, 745, 721, 743), cv::Vec4f(565, 424, 569, 412),
      cv::Vec4f(832, 432, 853, 438), cv::Vec4f(279, 947, 318, 941), cv::Vec4f(307, 967, 322, 984),
      cv::Vec4f(533, 236, 540, 242), cv::Vec4f(944, 372, 958, 367), cv::Vec4f(606, 137, 615, 141),
      cv::Vec4f(553, 389, 555, 398), cv::Vec4f(648, 370, 640, 380), cv::Vec4f(385, 974, 371, 974),
      cv::Vec4f(514, 269, 519, 256), cv::Vec4f(549, 600, 528, 594), cv::Vec4f(283, 1011, 277, 984),
      cv::Vec4f(328, 631, 313, 635), cv::Vec4f(336, 971, 333, 960), cv::Vec4f(254, 1036, 222, 1039),
      cv::Vec4f(631, 698, 639, 695), cv::Vec4f(569, 518, 609, 534), cv::Vec4f(344, 627, 330, 630),
      cv::Vec4f(522, 241, 539, 215), cv::Vec4f(258, 725, 259, 693), cv::Vec4f(644, 877, 747, 859),
      cv::Vec4f(554, 598, 553, 571) };

  LineSegmentDetector *lsd = new aLSD();

  cv::Mat yud_image = cv::imread(std::string(YUD_PATH)+"000.jpg");
  cv::Mat eurasian_image = cv::imread(std::string(EURASIAN_PATH)+"000.jpg");

  std::vector<cv::Vec4f> ls_yud = lsd->applyLSDetector(yud_image);
  std::vector<cv::Vec4f> ls_eurasian = lsd->applyLSDetector(eurasian_image);

  float closeness = 0.1;
  for (int i = 0; i < 50; i++) {
    BOOST_CHECK_CLOSE((int)ls_yud[i][0], gt_ls_yud[i][0], closeness);
    BOOST_CHECK_CLOSE((int)ls_yud[i][1], gt_ls_yud[i][1], closeness);
    BOOST_CHECK_CLOSE((int)ls_yud[i][2], gt_ls_yud[i][2], closeness);
    BOOST_CHECK_CLOSE((int)ls_yud[i][3], gt_ls_yud[i][3], closeness);
  }

  for (int i = 0; i < 50; i++) {
    BOOST_CHECK_CLOSE((int)ls_eurasian[i][0], gt_ls_eurasian[i][0], closeness);
    BOOST_CHECK_CLOSE((int)ls_eurasian[i][1], gt_ls_eurasian[i][1], closeness);
    BOOST_CHECK_CLOSE((int)ls_eurasian[i][2], gt_ls_eurasian[i][2], closeness);
    BOOST_CHECK_CLOSE((int)ls_eurasian[i][3], gt_ls_eurasian[i][3], closeness);
  }


  // VISUAL DEBUG
  // cv::Scalar color(0,255,0);
  // for (int i = 0; i < ls_yud.size(); i++) {
  //   cv::Point2f point1(ls_yud[i][0], ls_yud[i][1]);
  //   cv::Point2f point2(ls_yud[i][2], ls_yud[i][3]);
  //   cv::line(yud_image, point1, point2, color, yud_image.rows*0.001);
  // }
  //
  // for (int i = 0; i < ls_eurasian.size(); i++) {
  //   cv::Point2f point1(ls_eurasian[i][0], ls_eurasian[i][1]);
  //   cv::Point2f point2(ls_eurasian[i][2], ls_eurasian[i][3]);
  //   cv::line(eurasian_image, point1, point2, color, eurasian_image.rows*0.001);
  // }
  //
  // cv::imshow("YUD IMAGE" ,yud_image);
  // cv::imshow("EUASIAN IMAGE" ,eurasian_image);
  // cv::waitKey();

}

BOOST_AUTO_TEST_CASE(lineSegmentTime_testCase){

  std::vector<double> gt_times = {0.01,
                                  0.16};

  LineSegmentDetector *lsd = new aLSD();
  std::vector<double> sessions_time(lsd->getTimeSessionValues().size(), 0);

  int number_images = 100;
  for (int i = 0; i < number_images; i++) {
    std::string str_num = numberToString(number_images, 3);
    cv::Mat yud_image = cv::imread(std::string(YUD_PATH)+str_num+".jpg");
    std::vector<cv::Vec4f> ls_yud = lsd->applyLSDetector(yud_image);
    for(uint j = 0; j < sessions_time.size(); j++)
      sessions_time[j] += lsd->getTimeSessionValues()[j];
  }

  float closeness = 10.0;
  BOOST_CHECK_CLOSE(sessions_time[2]/number_images, gt_times[0], closeness);
  BOOST_CHECK_CLOSE(sessions_time[3]/number_images, gt_times[1], closeness);

  // Terminal print DEBUG
  // for(uint j = 0; j < sessions_time.size(); j++){
  //   std::cout << "Session " << lsd->getTimeSessionNames()[j]
  //             << " " << sessions_time[j]/number_images << std::endl;
  // }

}
