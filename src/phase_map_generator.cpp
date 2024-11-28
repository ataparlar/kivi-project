#include "kivi_project/phase_map_generator.hpp"
#include <math.h>

namespace kivi_project {

cv::Mat PhaseMapGenerator::generate_pattern(float phase_angle) {
    // double period_in_degree = period_in_px * 360 / width;
    // double period_in_radian = M_PI * period_in_degree / 180;  // change in 10px in radians
    double frequency = 2 * M_PI / period_in_px;
    double phase_angle_in_radian =  M_PI * phase_angle / 180;  // phase_angle in radians

    cv::Mat pattern(height, width, CV_32F);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double phase0 = frequency * x + phase_angle_in_radian; 
            pattern.at<float>(y, x) = 0.5f * (1.0f + std::sin(phase0));
        }
    }

    return pattern;
}

cv::Mat PhaseMapGenerator::compute_phase_map(
    const cv::Mat & pattern1, const cv::Mat & pattern2, const cv::Mat & pattern3
) {
    
    // auto I0 = generate_pattern(0);
    // auto I60 = generate_pattern(60);
    // auto I120 = generate_pattern(120);
    // auto I180 = generate_pattern(180);
    // auto I240 = generate_pattern(240);
    // auto I300 = generate_pattern(3000);


    cv::Mat phase_map(height, width, CV_32F);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double pattern1_val = pattern1.at<float>(y, x);
            double pattern2_val = pattern2.at<float>(y, x);
            double pattern3_val = pattern3.at<float>(y, x);
            double phase = std::atan2(
                std::sqrt(3.0f) * (pattern1_val - pattern3_val), 
                2.0f * pattern2_val - pattern1_val - pattern3_val
            );
            phase_map.at<float>(y, x) = phase;
        }
    }

    return phase_map;
}



} // namespace kivi_project
