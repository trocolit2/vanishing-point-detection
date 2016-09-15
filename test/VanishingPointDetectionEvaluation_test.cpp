#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VanishingPointDetectionEvaluation_test"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <vp_detectors/VanishingPointDetectionEvaluation.hpp>

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
      cv::Point2f(width*0.2, height*0.9), cv::Point2f(width*0.45, height*-0.1),
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
    (*line_id_by_vp) = {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4};

    return out_points;
  };

};


BOOST_AUTO_TEST_CASE(runEvaluation_testCase){

  std::vector<double> gt_error_yud = {0.1294138604208059, 0.09374071711040277,
                    0.238740094985235, 0.1066392239395906, 0.2452564863979747,
                    0.1940918562467152, 0.2717146243337353, 0.2081975265364146,
                    0.2359307272011255, 0.3993392573338072, 0.3615491886632097,
                    0.1886163665433818, 0.2206714348561189, 0.2004163752373303,
                    0.2404935843990407, 0.1918185530646206, 0.2946599335742697,
                    0.2538554803675743, 0.1762871409572875, 0.2529246349640139,
                    0.2308504060199617, 0.2369955047452947, 0.2357760406743347,
                    0.1759144460639443, 0.2002525959711799, 0.3634267834983763,
                    0.2800486283233735, 0.7056330413855718, 0.5065373111700873,
                    0.5644430776931623, 0.2123450618201548, 0.1013397852700812,
                    0.3831180576167725, 0.3307989156069466, 0.2815089862985939,
                    0.35699422038173, 0.2713184726154059, 0.2860445342229261,
                    0.2902958508898065, 0.2949012795190974, 0.1450793584797306,
                    0.1477833464934862, 0.1517387422885886, 0.09951673933231067,
                    0.2170740161929351, 0.06556695258035075, 0.160671682076373,
                    0.271370192315086, 0.2892354581799311, 0.2668079002216009,
                    0.2653300324303366, 0.270882892475862, 0.2334527028240146,
                    0.1996495567467377, 0.2627101884212233, 0.1841485998805639,
                    0.1821959151426924, 0.4288394606530955, 0.2575488083556947,
                    0.2246655179682041, 0.2343071281210992, 0.1412831594429118,
                    0.1884082132203693, 0.1044463459364857, 0.06994743235276728,
                    0.08715019239126108, 0.1420551284732564, 0.2263483688611435,
                    0.1103937137186948, 0.1028985359396674, 0.09217427476749893,
                    0.1414957698673488, 0.1141604718386238, 0.148314986505182,
                    0.1160761511544093, 0.3323953273525868, 0.3301866526260172,
                    0.3171833708295476, 0.4016211823475488, 0.1609663031192607,
                    0.1462095874621341, 0.07198365701230358, 0.1232179645123407,
                    0.1788015978486467, 0.1065860097637076, 0.3002155236206647,
                    0.2524330424717258, 0.08331915416887052, 0.136012903885363,
                    0.2798576344247016, 0.2202031482674355, 0.1760388676949426,
                    0.2598761561913739, 0.1999649017054305, 0.1316058466670578,
                    0.1588302644351078, 0.101691244312645, 0.1179175708677673,
                    0.1035889947335882, 0.0982684478481015, 0.08121045446259866,
                    0.1773399378758475};

std::vector<double> gt_error_eurasian = {0.2309990836121188, 0.2947195340029062,
                   0.1844226511370684, 0.3363576819191967, 0.2076526259916229,
                   0.1934689298125824, 0.1336579744112147, 0.09425679729357782,
                   0.05314771331852481, 0.363479683118048, 0.1893776025372944,
                   0.152046086701584, 0.06793951950397777, 0.1654773326670423,
                   0.2490063098344603, 0.07534496064217493, 0.4444965524061792,
                   0.1091077734158201, 0.07049135431092772, 0.2739533735171991,
                   0.172110553484159, 0.4415725499765988, 0.1759824754871985,
                   0.1244930012249467, 0.2624561808105674, 0.1123156556011822,
                   0.3316444986848791, 0.1882330610469014, 0.2395871894636672,
                   0.1654210100746765, 0.2068788192548431, 0.1356405034362477,
                   0.03925746750324164, 0.1648646440478263, 0.1010221817296097,
                   0.1055109112459604, 0.1648581037554601, 0.1532404017095197,
                   0.2977913661919866, 0.1336040098775402, 0.1171542403035559,
                   0.1747978509573363, 0.2932964611359785, 0.0966893843081091,
                   0.492224125113348, 0.1768325659041165, 0.1782159766305586,
                   0.06688722012083705, 0.1785424926186756, 0.2171349548032367,
                   0.2369634741872837, 0.09898193814018631, 0.1741174561903012,
                   0.1792965955898037, 0.2347036600454104, 0.1222607699328426,
                   0.1131263967993335, 0.2224769037292781, 0.3374522944397316,
                   0.182615754999627, 0.1439034168518156, 0.1364737722772665,
                   0.2818448169954662, 0.1073816204748827, 0.3123472900257867,
                   0.3321836009832965, 0.2305064628478386, 0.05222217214196703,
                   0.09337139061183798, 0.1011471365030491, 0.07834959066894423,
                   0.1800069043371797, 0.1020493900218437, 0.2758305059866238,
                   0.2307291015096763, 0.1073596080299483, 0.07356381476347408,
                   0.1082907897296765, 0.2687372007451793, 0.2057142835778613,
                   0.09842058100279298, 0.06767803287476054, 0.1152733954046019,
                   0.06231466935523507, 0.2507439214192141, 0.1152178039417674,
                   0.2259875562391869, 0.1794776520431764, 0.2825031267403829,
                   0.09123114725706906, 0.1298808477726384, 0.1579258749200806,
                   0.1003907912434786, 0.1397067335875108, 0.1923211975554038,
                   0.266691323968243, 0.08756764726124838, 0.1023734395041879,
                   0.1723665173017708, 0.2319331761406602, 0.2063996379061783,
                   0.4409940871460798, 1.397068740103252};


  VanishingPointDetectionEvaluation yud_evaluation( "YUD", YUD_PATH);
  VanishingPointDetectionEvaluation eurasian_evaluation( "EURASIAN",
                                                         EURASIAN_PATH);

  VanishingPointDetection *mock_detector = new MOCK_VPD();

  std::vector<double> error_yud, error_eurasian;
  error_yud = yud_evaluation.runEvaluation(mock_detector);
  error_eurasian = eurasian_evaluation.runEvaluation(mock_detector);

  for (uint i = 0; i < error_yud.size(); i++)
    BOOST_CHECK_CLOSE(error_yud[i], gt_error_yud[i], 0.00001);

  for (uint i = 0; i < error_eurasian.size(); i++)
    BOOST_CHECK_CLOSE(error_eurasian[i], gt_error_eurasian[i], 0.00001);

  free(mock_detector);
}


BOOST_AUTO_TEST_CASE(loadGroundTruth_testCase){


  std::vector<cv::Point2f> gt_zeniths =
                                {cv::Point2f(607.228, -16572.6),
                                 cv::Point2f(519.135, -8432.85)};
  std::vector<cv::Point3f> gt_horizonlines =
                            {cv::Point3f(0.0178457, -1, 269.868),
                             cv::Point3f(0.0120536, -0.999927, 567.297)};

  std::vector<int> gt_size = {102, 103};

  int reference_index = 11;
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
