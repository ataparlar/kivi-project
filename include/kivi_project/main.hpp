#include <iostream>
#include <kivi_project/phase_map_generator.hpp>

namespace kivi_project {

class KiviProject {

public:
explicit KiviProject();
   
void double_three_step_phase_shift();

private:
    PhaseMapGenerator::SharedPtr phase_map_generator_;

};

}

