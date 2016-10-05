// Method based on paper Robust Camera Self-Calibration from Monocular Images
// of Manhattan Worlds, 2012. Class name is the paper acronym: RCSCfMIoMW.
//
#include <VanishingPointDetection.hpp>

namespace vanishing_point {

class RCSCfMIoMW : public VanishingPointDetection{

  public:
    RCSCfMIoMW(uint ransac_iterations = 500, double error_threshold = 0.1);

    std::vector<cv::Point2f> applyVPDetector(
                                  cv::Mat image,
                                  std::vector<cv::Vec4f> lines_segments,
                                  std::vector<int> *line_id_by_vp = 0);

    uint getRansacIterations();
    void setRansacIterations(uint ransac_iterations);

    uint getErrorThreshold();
    void setErrorThreshold(double error_threshold);

    double getFocus();
    cv::Mat1f getRotationMatrix();


    private:
      double focus_;
      cv::Mat1f rotation_matrix_;
      uint ransac_iterations_;
      double error_threshold_;
};
}
