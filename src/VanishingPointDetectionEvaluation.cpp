#include <VanishingPointDetectionEvaluation.hpp>
#include <iostream>

namespace vanishing_point {

#define VPDE                  VanishingPointDetectionEvaluation::
#define YML_GT_FILE           "gt_data.yml"
#define GT_ZENITH_DATA_NAME   "zeniths"
#define GT_HORIZON_DATA_NAME  "horizon_lines"


VPDE VanishingPointDetectionEvaluation( std::string dataset_name,
                                        std::string dataset_path){
  _dataset_name = dataset_name;
  _dataset_path = dataset_path;
  loadGroundTruth(&_gt_zeniths, &_gt_horizon_lines);
}

void VPDE loadGroundTruth( std::vector<cv::Point2f> *gt_zeniths,
                           std::vector<cv::Point3f> *gt_horizon_lines){

  std::string gt_path = _dataset_path + std::string(YML_GT_FILE);

  cv::Mat raw_horizon_lines_gt, raw_zenith_gt;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs[ GT_HORIZON_DATA_NAME ] >> raw_horizon_lines_gt;
  fs[ GT_ZENITH_DATA_NAME ] >> raw_zenith_gt;

  cv::Mat1f mat_HL(raw_horizon_lines_gt);
  cv::Mat1f mat_Z(raw_zenith_gt);

  for (unsigned int k = 0; k < mat_HL.rows; k++) {

    cv::Point3f horizon_line( mat_HL[k][0], mat_HL[k][1], mat_HL[k][2]);
    cv::Point2f zenith(mat_Z[k][0], mat_Z[k][1]);

    gt_zeniths->push_back(zenith);
    gt_horizon_lines->push_back(horizon_line);
  }
}

std::vector<double> VPDE runEvaluation( VanishingPointDetection detector,
                                        bool show_detection){

  std::vector<double> error_vector(_gt_zeniths.size());
  for(uint i=0; i<error_vector.size(); ++i){

    // load image
    cv::Mat3b image = cv::imread("path");

    // detection output;
    std::vector<cv::Point2f> detected_vps;
    std::vector<int> line_by_vp;
    std::vector<cv::Vec4f> lines_segments;

    detected_vps = detector.applyVPDetector( image, &line_by_vp,
                                                    &lines_segments );
    cv::Point2f zenith = detected_vps[1];
    std::vector<cv::Point2f> horizon_points; = detected_vps[1];
    horizon_points.push_back(detected_vps[0]);
    for (uint j = 2; j < detected_vps.size(); j++) {
      horizon_points.push_back(detected_vps[i]);
    }

  }

}




const std::vector<cv::Point2f> VPDE getGTZeniths(){
  return _gt_zeniths;
}

const std::vector<cv::Point3f> VPDE getGTHorizonLines(){
  return _gt_horizon_lines;
}

}
