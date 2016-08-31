#include <opencv2/core/core.hpp>
#include <vector>

namespace vanishing_point {

cv::Mat drawOrthogonalVP(cv::Mat image, std::vector<cv::Point2f> points,
                         cv::Point2f principal_point = cv::Point2f(-1, -1));

cv::Mat drawHorizonLine(cv::Mat image, cv::Point3f line);

cv::Mat drawZenithLine(cv::Mat original_image, cv::Point2f zenith_point,
                       cv::Point2f principal_point,
                       cv::Point2f intersection_point);

cv::Point3f adjustPointsToDraw( cv::Point2f zenith_point,
                                cv::Point2f principal_point,
                                cv::Point3f horizon_line,
                                cv::Point2f *intersection_point,
                                cv::Point2f *zenith_local);


cv::Point3f defineEuclidianLineBy2Points(cv::Point2f point_inital,
                                         cv::Point2f point_final);

cv::Point2f definePointByEuclidianLinesIntersection(cv::Point3f line_initial,
                                                    cv::Point3f line_final);




// basead on http://graphics.cs.msu.ru/files/text/IJCV2011tretyak.pdf
// How to estimate error by horizon line
//   We therefore focused on the accuracy of the horizon estimation.
// Assume that the horizon is given as a (linear) function H(x) of a
// pixel  x-coordinate. Assume that H0(x) and H1(x) are the ground truth and
// the estimated horizon. We de ne the estimation error as the maximum
// euclidean distance between the lines H0(x) and H1(x) within the image
// domain (0 < x < image width),  divided  by  the  image  height.
double normalizedDistanceBetweenHorizonLines( cv::Point3f horizon_line,
                                              cv::Point3f gt_horizon_line,
                                              cv::Size image_size );



// basead on http://graphics.cs.msu.ru/files/text/IJCV2011tretyak.pdf
// How estimate the horizon line
//   After running each method we obtain the zenith, as well as a number of
// vanishing points corresponding to the parallel families of the line segments
// (for baseline methods) or lines (for our method). We use this information to
// estimate the position of the horizon in an image. The horizon is estimated
// in the same way for all methods. Thus, we restrict it to be perpendicular to
// the line connecting principal point and zenith. So the slope of horizon is
// given by zenith and we estimate only its position along the 1D axis. To do
// this last step, we perform the weighted least squares fit, where the weight
// of each detected horizontal vanishing point equals the number of
// corresponding lines (or line segments). In the case when no horizontal
// vanishing points or zenith were found, the horizon was drawn strongly
// horizontally in the middle of the image. In our approach the case when no
// zenith is found is impossible, as zenith is the part of our model.
// In contrast, in the other competing methods the horizontal vanishing points
// and the zenith are treated equally.
cv::Point3f horizonLineEstimation(cv::Point2f zenith,
                                  cv::Point2f principal_point,
                                  std::vector<cv::Point2f> vanishing_points,
                                  std::vector<int> lines_by_vp,
                                  cv::Point3f *horizon_without_zenith = 0);

}
