#include <iostream>
#include "opencv2/opencv.hpp"

namespace kivi_project {

class PhaseMapGenerator {

public:
using SharedPtr = std::shared_ptr<PhaseMapGenerator>;
using ConstSharedPtr = const std::shared_ptr<PhaseMapGenerator>;

cv::Mat generate_pattern(float phase_angle);
cv::Mat compute_phase_map();

private:
    int width = 1280;
    int height = 720;
    float period_in_px = 10;
};

}  // namespace kivi_project
