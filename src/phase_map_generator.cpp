#include "kivi_project/phase_map_generator.hpp"
#include <math.h>

namespace kivi_project {

cv::Mat PhaseMapGenerator::generate_pattern(float phase_angle) {
    float frequency = M_PI * period_in_px / 180;  // change in 10px in radians
    double phase_angle_in_radian =  M_PI * phase_angle / 180;  // phase_angle in radians

    cv::Mat pattern(height, width, CV_32F);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double phase0 = frequency * x + phase_angle_in_radian; 
            pattern.at<float>(y, x) = 0.5f * (1.0f + std::sin(phase0));
            // std::cout << "phase: " << phase0 << std::endl;
            // std::cout << "pattern.at<float>(y, x): " << pattern.at<float>(y, x) << std::endl;
        }
    }

    return pattern;
}

cv::Mat PhaseMapGenerator::compute_phase_map() {
    
    auto I0 = generate_pattern(0);
    auto I120 = generate_pattern(120);
    auto I240 = generate_pattern(240);


    cv::Mat phase_map(height, width, CV_32F);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double I0_val = I0.at<float>(y, x);
            double I120_val = I120.at<float>(y, x);
            double I240_val = I240.at<float>(y, x);
            double phase = std::atan2(
                std::sqrt(3.0f) * (I0_val - I240_val), 
                2.0f * I120_val - I0_val - I240_val
            );
            phase_map.at<float>(y, x) = phase;
        }
    }

    return phase_map;
}



} // namespace kivi_project
