#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point{

class LineSegmentDetector{

  public:
    virtual std::vector<cv::Vec4f> applyLSDetector(cv::Mat image) = 0;

    const std::vector<std::string> getTimeSessionNames(){
      return _time_sessions;
    };
    const std::vector<double> getTimeSessionValues(){
      return _time_values;
    };

  protected:
    std::vector<std::string> _time_sessions;
    std::vector<double> _time_values;

};
}
