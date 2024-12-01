#include "kivi_project/main.hpp"

namespace kivi_project {

KiviProject::KiviProject() {
      phase_map_generator_ = std::make_shared<PhaseMapGenerator>();
}


void KiviProject::double_three_step_phase_shift() {
    cv::Mat pattern1 = phase_map_generator_->generate_pattern(0);
    cv::Mat pattern2 = phase_map_generator_->generate_pattern(60);
    cv::Mat pattern3 = phase_map_generator_->generate_pattern(120);

    cv::Mat phase_map1 = phase_map_generator_->compute_phase_map(
        pattern1, pattern2, pattern3);

    cv::Mat pattern4 = phase_map_generator_->generate_pattern(180);
    cv::Mat pattern5 = phase_map_generator_->generate_pattern(240);
    cv::Mat pattern6 = phase_map_generator_->generate_pattern(300);

    cv::Mat phase_map2 = phase_map_generator_->compute_phase_map(
        pattern4, pattern5, pattern6);

    cv::Mat average_phase_map = phase_map_generator_->average_of_phase_maps(
        phase_map1, phase_map2);

    cv::imshow("Display window", average_phase_map);
    cv::waitKey(0);

    cv::Mat normalized_phase_map;
    cv::normalize(average_phase_map, normalized_phase_map, 0, 255, cv::NORM_MINMAX);
    cv::imwrite(
        "/home/ataparlarl/projects/kivi_assignment/kivi_project/export/average_phase_map.png", 
        normalized_phase_map);

}

}


int main(int, char**){

    kivi_project::KiviProject kivi;
    
    kivi.double_three_step_phase_shift();
    

}
