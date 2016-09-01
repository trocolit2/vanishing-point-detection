#include <VanishingPointDetectionEvaluation.hpp>

namespace vanishing_point {

#define VPDE                  VanishingPointDetectionEvaluation::
#define YML_GT_FILE           "gt_data.yml"
#define GT_ZENITH_DATA_NAME   "zeniths"
#define GT_HORIZON_DATA_NAME  "horizon_lines"


VPDE VanishingPointDetectionEvaluation( std::string dataset_name,
                                        std::string dataset_path){
  _dataset_name = dataset_name;
  _dataset_path = dataset_path;
  loadGroundTruth(_gt_zeniths, _gt_horizon_lines);
}

void VPDE loadGroundTruth( std::vector<cv::Point2f> &gt_zeniths,
                           std::vector<cv::Point3f> &gt_horizon_lines){

  std::string gt_path = _dataset_path + std::string(YML_GT_FILE);

  cv::Mat1f raw_horizon_lines_gt, raw_zenith_gt;
  cv::FileStorage fs(gt_path, cv::FileStorage::READ);
  fs[ GT_HORIZON_DATA_NAME ] >> raw_horizon_lines_gt;
  fs[ GT_ZENITH_DATA_NAME ] >> raw_zenith_gt;

  for (unsigned int k = 0; k < raw_horizon_lines_gt.rows; k++) {

    cv::Point3f horizon_line( raw_horizon_lines_gt[k][0],
                              raw_horizon_lines_gt[k][1],
                              raw_horizon_lines_gt[k][2]);

    cv::Point2f zenith(raw_zenith_gt[k][0], raw_zenith_gt[k][1]);

    gt_zeniths.push_back(zenith);
    gt_horizon_lines.push_back(horizon_line);
  }
}

}
