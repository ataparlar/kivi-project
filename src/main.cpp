#include "kivi_project/main.hpp"

namespace kivi_project {

KiviProject::KiviProject() {
      phase_map_generator_ = std::make_shared<PhaseMapGenerator>();
}


void KiviProject::kivi() {
    // cv::Mat image = phase_map_generator_->generate_pattern(0);

    // cv::Mat pattern1 = phase_map_generator_->generate_pattern(0);
    // cv::Mat pattern2 = phase_map_generator_->generate_pattern(60);
    // cv::Mat pattern3 = phase_map_generator_->generate_pattern(120);

    cv::Mat pattern1 = phase_map_generator_->generate_pattern(180);
    cv::Mat pattern2 = phase_map_generator_->generate_pattern(240);
    cv::Mat pattern3 = phase_map_generator_->generate_pattern(300);

    cv::Mat phase_map = phase_map_generator_->compute_phase_map(
        pattern1, pattern2, pattern3);

    cv::imshow("Display window", phase_map);
    cv::waitKey(0);

    cv::Mat normalized_phase_map;
    cv::normalize(phase_map, normalized_phase_map, 0, 255, cv::NORM_MINMAX);
    cv::imwrite(
        "/home/ataparlarl/projects/kivi_assignment/kivi_project/export/phase_map2.png", 
        normalized_phase_map);

}

}


int main(int, char**){

    kivi_project::KiviProject kivi;
    
    kivi.kivi();
    

}
