// Method based on paper Robust Camera Self-Calibration from Monocular Images
// of Manhattan Worlds, 2012. Class name is the paper acronym: RCSCfMIoMW.
//
#include <VanishingPointDetection.hpp>

namespace vanishing_point {

class RCSCfMIoMW : public VanishingPointDetection{

  public:
    RCSCfMIoMW();

    std::vector<cv::Point2f> applyVPDetector(
                                  cv::Mat image,
                                  std::vector<cv::Vec4f> lines_segments,
                                  std::vector<int> *line_id_by_vp = 0);

    double getFocus();
    cv::Mat1f getRotationMatrix();

    private:
      double focus_;
      cv::Mat1f rotation_matrix_;


};
}
