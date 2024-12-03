#include "opencv2/opencv.hpp"

#include <iostream>

namespace kivi_project
{

class PhaseMapGenerator
{
public:
  using SharedPtr = std::shared_ptr<PhaseMapGenerator>;
  using ConstSharedPtr = const std::shared_ptr<PhaseMapGenerator>;

  cv::Mat generate_pattern(float phase_angle);
  cv::Mat compute_phase_map(
    const cv::Mat & pattern1, const cv::Mat & pattern2, const cv::Mat & pattern3);
  cv::Mat average_of_phase_maps(const cv::Mat & phase_map1, const cv::Mat & phase_map2);

private:
  int width = 1280;
  int height = 720;
  float period_in_px = 10;
};

}  // namespace kivi_project
