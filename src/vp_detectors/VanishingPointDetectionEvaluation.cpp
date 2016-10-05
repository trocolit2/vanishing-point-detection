#include <VanishingPointDetectionEvaluation.hpp>
#include <VanishingPointDetectionTools.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string> //

namespace vanishing_point {

#define VPDE                  VanishingPointDetectionEvaluation::
#define YML_GT_FILE           "gt_data.yml"
#define GT_ZENITH_DATA_NAME   "zeniths"
#define GT_HORIZON_DATA_NAME  "horizon_lines"

VPDE VanishingPointDetectionEvaluation( std::string dataset_name,
                                        std::string dataset_path,
                                        std::string image_prefix,
                                        std::string image_sufix ){
  _dataset_name = dataset_name;
  _dataset_path = dataset_path;
  _image_prefix = image_prefix;
  _image_sufix = image_sufix;
  loadGroundTruth( dataset_path, &_gt_zeniths, &_gt_horizon_lines);
}



void VPDE loadGroundTruth( std::string dataset_path,
                           std::vector<cv::Point2f> *gt_zeniths,
                           std::vector<cv::Point3f> *gt_horizon_lines){

  std::string gt_path = dataset_path + std::string(YML_GT_FILE);

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


std::vector<double> VPDE runEvaluation( VanishingPointDetection *detector){

  // set path with sequence of images
  std::string str_size = std::to_string(_gt_zeniths.size());
  str_size = std::to_string( str_size.size());
  str_size = "%0"+str_size+"d";
  std::string seq_path = _dataset_path+_image_prefix+str_size+_image_sufix;
  cv::VideoCapture capture(seq_path);

  std::vector<double> error_vector(_gt_zeniths.size());
  for(uint i=0; i < error_vector.size(); ++i){

    // load image
    cv::Mat3b image;
    capture >> image;

    // detection output;
    std::vector<cv::Point2f> detected_vps;
    std::vector<int> lines_by_vp;
    std::vector<cv::Vec4f> lines_segments;

    detected_vps = detector->applyVPDetector( image, lines_segments ,
                                              &lines_by_vp);
    // get zenith point
    cv::Point2f zenith = detected_vps[1];

    // get others points
    std::vector<cv::Point2f> horizon_points;
    horizon_points.push_back(detected_vps[0]);
    for (uint j = 2; j < detected_vps.size(); j++)
      horizon_points.push_back(detected_vps[j]);

    // count lines per horizontal vanishing_point
    std::vector<int> numbers_lines_per_vp(horizon_points.size());
    for(uint k = 0; k < lines_by_vp.size(); k++)
      if( lines_by_vp[k] == 0)
        numbers_lines_per_vp[ lines_by_vp[k] ]++;
      else if(lines_by_vp[k] != 1)
        numbers_lines_per_vp[ lines_by_vp[k]-1 ]++;

    // estimating horizon line
    cv::Point2f principal_point(image.cols/2, image.rows/2);
    cv::Point3f horizon_line = horizonLineEstimation( zenith, principal_point,
                                                      horizon_points,
                                                      numbers_lines_per_vp);

    // estimating error
    cv::Point2f max_point1, max_point2;
    error_vector[i] = normalizedMaxDistanceBetweenHorizonLines(
                                                        horizon_line,
                                                        _gt_horizon_lines[i],
                                                        image.size(),
                                                        &max_point1,
                                                        &max_point2);

    // Visual Debug
    // cv::Point2f zenith_local, intersection_point;
    // adjustPointsToDraw( zenith, principal_point, horizon_line,
    //                     &intersection_point, &zenith_local);
    //
    // image = drawHorizonLine(image, horizon_line);
    // image = drawHorizonLine(image, _gt_horizon_lines[i], cv::Scalar(255,255,0));
    // image = drawZenithLine( image, zenith_local, principal_point,
    //                         intersection_point);
    // cv::Scalar color (255, 0, 0);
    // for (int l = 0; l < horizon_points.size(); l++) {
    //   cv::circle(image, horizon_points[l], image.rows * 0.01, color, -1);
    // }
    //   cv::line(image, max_point1, max_point2, color, image.rows * 0.005 );
    //
    // cv::imshow("out horizon",image);
    // cv::waitKey();

  }
  return error_vector;
}




const std::vector<cv::Point2f> VPDE getGTZeniths(){
  return _gt_zeniths;
}

const std::vector<cv::Point3f> VPDE getGTHorizonLines(){
  return _gt_horizon_lines;
}

}
