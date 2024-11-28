#include <iostream>
#include <kivi_project/phase_map_generator.hpp>

namespace kivi_project {

class KiviProject {

public:
explicit KiviProject();
   
void kivi();

private:
    PhaseMapGenerator::SharedPtr phase_map_generator_;

};

}

