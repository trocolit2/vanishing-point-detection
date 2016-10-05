#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point{

class VanishingPointDetection{

  public:
    virtual std::vector<cv::Point2f> applyVPDetector(
                                    cv::Mat image,
                                    std::vector<cv::Vec4f> lines_segments,
                                    std::vector<int> *line_id_by_vp = 0) = 0;

  protected:
    std::vector<std::string> time_sessions;
    std::vector<double> time_values;


};
}
