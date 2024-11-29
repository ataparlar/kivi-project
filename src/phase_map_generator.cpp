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

cv::Mat PhaseMapGenerator::average_of_phase_maps(
    const cv::Mat & phase_map1, const cv::Mat & phase_map2
) {
    /* We need to create complex versions of the phase maps.
    Convert them from polar to cartesian in order to take average
    Each phase map has real part and imaginary part due to the
    signal processing basics. So, make 2 channels. */

    cv::Mat phase_map1_complex(phase_map1.size(), CV_32FC2);
    cv::Mat phase_map2_complex(phase_map2.size(), CV_32FC2);

    // For holding real and imaginary part separate
    cv::Mat phase_map1_planes[] = {cv::Mat(), cv::Mat()}; // cos and sin
    cv::Mat phase_map2_planes[] = {cv::Mat(), cv::Mat()}; // cos and sin

    cv::polarToCart(  // Seperate the phase_map1. 0-255 needs to normalized.
        cv::Mat::ones(phase_map1.size(), CV_32F), phase_map1, 
        phase_map1_planes[0], phase_map1_planes[1]);
    cv::polarToCart(
        cv::Mat::ones(phase_map2.size(), CV_32F), phase_map2,
        phase_map2_planes[0], phase_map2_planes[1]);

    // We are in cartesian planes or in real numbers now. We can take average.
    cv::Mat avgerage_cos = (phase_map1_planes[0] + phase_map2_planes[0]) / 2.0;
    cv::Mat avgerage_sin = (phase_map1_planes[1] + phase_map2_planes[1]) / 2.0;
    // cv::Mat average_phase_complex = (phase_map1_complex, phase_map2_complex) / 2.0;

    // Now convert this to polar back.
    cv::Mat average_phase, average_magnitude;
    cv::cartToPolar(avgerage_cos, avgerage_sin, average_magnitude, average_phase);

    return average_phase;
}



} // namespace kivi_project
