#include "kivi_project/main.hpp"

namespace kivi_project {

KiviProject::KiviProject() {
      phase_map_generator_ = std::make_shared<PhaseMapGenerator>();
}


void KiviProject::kivi() {
    // cv::Mat image = phase_map_generator_->generate_pattern(0);
    cv::Mat image = phase_map_generator_->compute_phase_map();

    cv::imshow("Display window", image);
    cv::waitKey(0);
}

}


int main(int, char**){

    kivi_project::KiviProject kivi;
    
    kivi.kivi();
    

}
