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


BOOST_AUTO_TEST_CASE(teste){
  std::cout<<" FIrst Test" <<std::endl;
}
