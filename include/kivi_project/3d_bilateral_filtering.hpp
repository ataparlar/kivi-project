#include <iostream>
#include <vector>
#include <memory>

namespace kivi_project {

class BilateralFiltering3D {

public:
    using SharedPtr = std::shared_ptr<BilateralFiltering3D>;
    using ConstSharedPtr = const std::shared_ptr<BilateralFiltering3D>;

    struct PointXYZ {
        float x;
        float y;
        float z;
    };
    struct NormalXYZ {
        float nx;
        float ny;
        float nz;
    };
    struct TriangleXYZ {
        PointXYZ point;
        NormalXYZ normal;
    };
    using CloudXYZ = std::vector<TriangleXYZ>;

    float euclideanDistance(const PointXYZ& a, const PointXYZ& b);
    float dotProduct(const NormalXYZ& a, const NormalXYZ& b);
    NormalXYZ normalize(const NormalXYZ& n);
    PointXYZ denoisePoint(
        const PointXYZ& v, const NormalXYZ& n, 
        const std::vector<PointXYZ>& neighborhood, 
        float sigma_c, float sigma_s);

    std::vector<PointXYZ> getKNearestNeighbors(
        const PointXYZ& v, 
        const CloudXYZ& allVertices, int k); 
    std::vector<std::vector<std::vector<float>>> readPLYFileWithNormals(
        const std::string& filename, 
        int width, int height);
    void writePLYFile(
        const std::string& filename, 
        const std::vector<std::vector<std::vector<float>>>& organized_point_cloud, 
        bool organized = false);

private:

};

}  // namespace kivi_project


