#include <iostream>
#include <vector>
#include <memory>

namespace kivi_project {

class BilateralFiltering3D {

public:
using SharedPtr = std::shared_ptr<BilateralFiltering3D>;
using ConstSharedPtr = const std::shared_ptr<BilateralFiltering3D>;

void writePLYFile(
    const std::string& filename, 
    const std::vector<std::vector<std::vector<float>>>& organized_point_cloud, 
    bool organized = false);
std::vector<std::vector<std::vector<float>>> readPLYFileWithNormals(
    const std::string& filename, 
    int width, int height);

private:
    int width = 1280;
    int height = 720;
    float period_in_px = 10;
};

}  // namespace kivi_project


