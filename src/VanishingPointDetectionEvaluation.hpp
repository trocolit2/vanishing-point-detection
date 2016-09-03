#include <opencv2/core/core.hpp>
#include <vector>
#include <VanishingPointDetection.hpp>

namespace vanishing_point {

class VanishingPointDetectionEvaluation{

  public:
    VanishingPointDetectionEvaluation( std::string dataset_name,
                                       std::string dataset_path);

    std::vector<double> runEvaluation(VanishingPointDetection detector,
                                      bool show_detection);

    std::string getDatasetName();
    std::string getDatasetPath();

    const std::vector<cv::Point2f> getGTZeniths();
    const std::vector<cv::Point3f> getGTHorizonLines();

    cv::Mat drawVPDetection(cv::Mat image, std::vector<cv::Point2f> points);

  private:
    std::string _dataset_name;
    std::string _dataset_path;
    std::vector<double> _vector_error_images;

    std::vector<cv::Point2f> _gt_zeniths;
    std::vector<cv::Point3f> _gt_horizon_lines;

    void loadGroundTruth( std::vector<cv::Point2f> *gt_zeniths,
                          std::vector<cv::Point3f> *gt_horizon_lines);


};

}
