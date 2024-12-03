#include <kivi_project/3d_bilateral_filtering.hpp>
#include <kivi_project/correspondence_matching.hpp>
#include <kivi_project/phase_map_generator.hpp>

#include <iostream>

namespace kivi_project
{

class KiviProject
{
public:
  explicit KiviProject();

  void double_three_step_phase_shift();
  void bilateral_filter_3d();
  void correspondence_matching();

private:
  PhaseMapGenerator::SharedPtr phase_map_generator_;
  BilateralFiltering3D::SharedPtr bilateral_filter_3d_;
  CorrespondenceMatching::SharedPtr correspondence_matching_;
};

}  // namespace kivi_project
