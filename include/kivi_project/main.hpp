#include <iostream>
#include <kivi_project/phase_map_generator.hpp>
#include <kivi_project/3d_bilateral_filtering.hpp>

namespace kivi_project {

class KiviProject {

public:
explicit KiviProject();
   
void double_three_step_phase_shift();
void bilateral_filter_3d();

private:
    PhaseMapGenerator::SharedPtr phase_map_generator_;
    BilateralFiltering3D::SharedPtr bilateral_filter_3d_;

};

}

