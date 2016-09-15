#include <opencv2/core/core.hpp>
#include <vector>
#include <vp_detectors/VanishingPointDetection.hpp>

namespace vanishing_point {

class VanishingPointDetectionEvaluation{

  public:
    VanishingPointDetectionEvaluation( std::string dataset_name,
                                       std::string dataset_path,
                                       std::string image_prefix = "",
                                       std::string image_sufix = ".jpg");

    std::vector<double> runEvaluation(VanishingPointDetection *detector);

    std::string getDatasetName();
    std::string getDatasetPath();

    const std::vector<cv::Point2f> getGTZeniths();
    const std::vector<cv::Point3f> getGTHorizonLines();

  private:
    std::string _dataset_name;
    std::string _dataset_path;
    std::string _image_prefix;
    std::string _image_sufix;

    // std::vector<double> _vector_error_images;

    std::vector<cv::Point2f> _gt_zeniths;
    std::vector<cv::Point3f> _gt_horizon_lines;

    void loadGroundTruth(  std::string dataset_path,
                           std::vector<cv::Point2f> *gt_zeniths,
                           std::vector<cv::Point3f> *gt_horizon_lines);




};

}
